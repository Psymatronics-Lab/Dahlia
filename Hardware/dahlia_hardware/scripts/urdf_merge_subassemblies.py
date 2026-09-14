#!/usr/bin/env python3
"""Postprocess an Onshape-exported URDF into a minimal rigid-body model.

The Onshape URDF exporter emits one link per part, wires the parts together with
`fixed` joints, and drops in massless "mate connector" links as proxy frames.
The result is kinematically correct but unusable: dozens of links that can never
move relative to each other, and dummy frames cluttering the tree.

This tool rewrites the model so that every link is an actual degree of freedom:

  1. Links joined by `fixed` joints (including Onshape part groups) are collapsed
     into a single subassembly link.  Mass, centre of mass and the full inertia
     tensor are combined with the parallel-axis theorem.
  2. The STL of every member part is baked into the subassembly frame and the
     meshes are concatenated into one STL per subassembly.
  3. Mate-connector proxy links are removed.  Where a mate connector sits between
     two real links, the joint is re-created directly between those links; where
     a mate connector pair is the only thing tying two subtrees together, a fixed
     joint is created between their parent links.  A proxy directly below a
     movable joint is absorbed into the body under it, which keeps the proxy's
     frame and takes the part's name -- see classify_mate_connectors() for why
     that case cannot be collapsed by composing transforms.
  4. Everything else is carried across verbatim: joint types, axes, limits,
     dynamics, mimic tags, materials and mesh geometry.

A verification pass re-derives the world-frame pose of every joint, axis and
visual plus the global mass/COM/inertia from the output and compares them
against the input, so the rewrite is checked rather than assumed.

Pure standard library: no numpy, no trimesh.

Usage:
    python urdf_merge_subassemblies.py path/to/robot_full/urdf/robot_full.urdf -o path/to/robot_sim

The output package holds only urdf/<robot name>.urdf and meshes/<link>.stl.
The package name, robot name and URDF file name all default to the output
directory name.

The dahlia_m1_full invocation, run from Hardware/dahlia_hardware/ -- writes the
merged package to dahlia_m1_sim/ next to the input, since -o defaults to the
input package with its _full suffix replaced by _sim:

    python scripts/urdf_merge_subassemblies.py dahlia_m1_full/urdf/dahlia_m1_full.urdf --name-map scripts/dahlia_m1_full_names.json

Re-run it after every Onshape export.  Onshape renumbers part instances between
exports, which is why the name map is keyed by driving joint rather than by link
name -- see apply_name_map().  Add -v to list the parts going into each fused
mesh, --no-fuse to keep one <visual> per part (and so per-part colours), and
--manifest to also write a JSON record of how the source links were merged.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Iterable, Sequence

Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]

IDENTITY3: Mat3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
ZERO3: Vec3 = (0.0, 0.0, 0.0)


# --------------------------------------------------------------------------- #
# Linear algebra
# --------------------------------------------------------------------------- #

def mat_mul(a: Mat3, b: Mat3) -> Mat3:
    return tuple(  # type: ignore[return-value]
        tuple(a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] for j in range(3))
        for i in range(3)
    )


def mat_vec(a: Mat3, v: Sequence[float]) -> Vec3:
    return (
        a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
        a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
        a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
    )


def mat_transpose(a: Mat3) -> Mat3:
    return ((a[0][0], a[1][0], a[2][0]),
            (a[0][1], a[1][1], a[2][1]),
            (a[0][2], a[1][2], a[2][2]))


def mat_det(a: Mat3) -> float:
    return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))


def mat_add(a: Mat3, b: Mat3) -> Mat3:
    return tuple(tuple(a[i][j] + b[i][j] for j in range(3)) for i in range(3))  # type: ignore


def congruence(r: Mat3, m: Mat3) -> Mat3:
    """R * M * R^T -- re-expresses tensor M in a frame rotated by R."""
    return mat_mul(mat_mul(r, m), mat_transpose(r))


def vec_sub(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_norm(v: Sequence[float]) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def orthonormalize(m: Mat3) -> Mat3:
    """Gram-Schmidt, to shed drift accumulated over long transform chains."""
    x = m[0][0], m[1][0], m[2][0]
    y = m[0][1], m[1][1], m[2][1]
    nx = vec_norm(x)
    if nx < 1e-12:
        return IDENTITY3
    x = (x[0] / nx, x[1] / nx, x[2] / nx)
    d = x[0] * y[0] + x[1] * y[1] + x[2] * y[2]
    y = (y[0] - d * x[0], y[1] - d * x[1], y[2] - d * x[2])
    ny = vec_norm(y)
    if ny < 1e-12:
        return IDENTITY3
    y = (y[0] / ny, y[1] / ny, y[2] / ny)
    z = (x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0])
    return ((x[0], y[0], z[0]), (x[1], y[1], z[1]), (x[2], y[2], z[2]))


def rpy_to_mat(roll: float, pitch: float, yaw: float) -> Mat3:
    """URDF fixed-axis convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def mat_to_rpy(m: Mat3) -> Vec3:
    """Inverse of rpy_to_mat, with the gimbal-lock branches handled."""
    m = orthonormalize(m)
    if abs(m[2][0]) < 1.0 - 1e-10:
        pitch = math.asin(max(-1.0, min(1.0, -m[2][0])))
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    elif m[2][0] <= -1.0 + 1e-10:      # pitch = +pi/2
        pitch, yaw = math.pi / 2.0, 0.0
        roll = math.atan2(m[0][1], m[0][2])
    else:                               # pitch = -pi/2
        pitch, yaw = -math.pi / 2.0, 0.0
        roll = math.atan2(-m[0][1], -m[0][2])
    return (roll, pitch, yaw)


@dataclass(frozen=True)
class Transform:
    """Rigid transform; `rot`/`pos` map a point in the child frame to the parent."""

    rot: Mat3 = IDENTITY3
    pos: Vec3 = ZERO3

    def __matmul__(self, other: "Transform") -> "Transform":
        return Transform(mat_mul(self.rot, other.rot),
                         tuple(a + b for a, b in zip(mat_vec(self.rot, other.pos), self.pos)))  # type: ignore

    def inverse(self) -> "Transform":
        rt = mat_transpose(self.rot)
        p = mat_vec(rt, self.pos)
        return Transform(rt, (-p[0], -p[1], -p[2]))

    def apply(self, v: Sequence[float]) -> Vec3:
        r = mat_vec(self.rot, v)
        return (r[0] + self.pos[0], r[1] + self.pos[1], r[2] + self.pos[2])

    def rotate(self, v: Sequence[float]) -> Vec3:
        return mat_vec(self.rot, v)

    @staticmethod
    def from_xyz_rpy(xyz: Sequence[float], rpy: Sequence[float]) -> "Transform":
        return Transform(rpy_to_mat(*rpy), (float(xyz[0]), float(xyz[1]), float(xyz[2])))

    def to_xyz_rpy(self) -> tuple[Vec3, Vec3]:
        return self.pos, mat_to_rpy(self.rot)


# --------------------------------------------------------------------------- #
# STL I/O
# --------------------------------------------------------------------------- #

_TRI = struct.Struct("<12fH")   # normal, v1, v2, v3, attribute byte count
_BINARY_HEADER = 84


@dataclass
class RawMesh:
    """A binary-STL triangle payload: `count` records of `_TRI` back to back."""

    count: int
    payload: bytes


def read_stl(path: str) -> RawMesh:
    with open(path, "rb") as handle:
        data = handle.read()
    if len(data) >= _BINARY_HEADER:
        count = struct.unpack_from("<I", data, 80)[0]
        if len(data) == _BINARY_HEADER + 50 * count:
            return RawMesh(count, data[_BINARY_HEADER:])
    return _parse_ascii_stl(data, path)


def _parse_ascii_stl(data: bytes, path: str) -> RawMesh:
    try:
        text = data.decode("utf-8", errors="replace")
    except Exception as exc:                                # pragma: no cover
        raise ValueError(f"{path}: not a readable STL") from exc
    if "facet" not in text:
        raise ValueError(f"{path}: neither a valid binary nor ASCII STL")

    numbers = re.compile(r"[-+0-9.eE]+")
    normal: Vec3 = ZERO3
    verts: list[Vec3] = []
    out = bytearray()
    count = 0
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("facet normal"):
            vals = [float(v) for v in numbers.findall(stripped[12:])]
            normal = tuple(vals[:3]) if len(vals) >= 3 else ZERO3  # type: ignore
            verts = []
        elif stripped.startswith("vertex"):
            vals = [float(v) for v in numbers.findall(stripped[6:])]
            verts.append(tuple(vals[:3]))                    # type: ignore
        elif stripped.startswith("endfacet"):
            if len(verts) == 3:
                out += _TRI.pack(*normal, *verts[0], *verts[1], *verts[2], 0)
                count += 1
    return RawMesh(count, bytes(out))


def write_stl(path: str, header: str, chunks: Iterable[bytes], count: int) -> None:
    head = header.encode("ascii", errors="replace")[:79].ljust(80, b"\0")
    with open(path, "wb") as handle:
        handle.write(head)
        handle.write(struct.pack("<I", count))
        for chunk in chunks:
            handle.write(chunk)


def transform_mesh(mesh: RawMesh, xf: Transform, scale: Vec3) -> tuple[bytes, list[Vec3]]:
    """Bake `xf` (and the URDF mesh `scale`) into the triangles.

    Returns the transformed payload and the axis-aligned bounds [lo, hi].  Facet
    normals are recomputed from the transformed winding -- the only definition
    that stays correct under non-uniform scale -- falling back to the rotated
    source normal for degenerate (zero-area) facets.  A mirroring scale flips
    the winding so that facets keep pointing outwards.
    """
    (r00, r01, r02), (r10, r11, r12), (r20, r21, r22) = xf.rot
    tx, ty, tz = xf.pos
    sx, sy, sz = scale

    # Linear part with the mesh scale folded in.
    m00, m01, m02 = r00 * sx, r01 * sy, r02 * sz
    m10, m11, m12 = r10 * sx, r11 * sy, r12 * sz
    m20, m21, m22 = r20 * sx, r21 * sy, r22 * sz
    mirrored = mat_det(((m00, m01, m02), (m10, m11, m12), (m20, m21, m22))) < 0.0

    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    out = bytearray(50 * mesh.count)
    pack_into = _TRI.pack_into
    offset = 0
    sqrt = math.sqrt

    for rec in _TRI.iter_unpack(mesh.payload):
        nx, ny, nz, ax, ay, az, bx, by, bz, cx, cy, cz, attr = rec

        pax = m00 * ax + m01 * ay + m02 * az + tx
        pay = m10 * ax + m11 * ay + m12 * az + ty
        paz = m20 * ax + m21 * ay + m22 * az + tz
        pbx = m00 * bx + m01 * by + m02 * bz + tx
        pby = m10 * bx + m11 * by + m12 * bz + ty
        pbz = m20 * bx + m21 * by + m22 * bz + tz
        pcx = m00 * cx + m01 * cy + m02 * cz + tx
        pcy = m10 * cx + m11 * cy + m12 * cz + ty
        pcz = m20 * cx + m21 * cy + m22 * cz + tz

        if mirrored:
            pbx, pby, pbz, pcx, pcy, pcz = pcx, pcy, pcz, pbx, pby, pbz

        ux, uy, uz = pbx - pax, pby - pay, pbz - paz
        vx, vy, vz = pcx - pax, pcy - pay, pcz - paz
        fx = uy * vz - uz * vy
        fy = uz * vx - ux * vz
        fz = ux * vy - uy * vx
        length = sqrt(fx * fx + fy * fy + fz * fz)
        if length > 1e-30:
            fx, fy, fz = fx / length, fy / length, fz / length
        else:
            fx = m00 * nx + m01 * ny + m02 * nz
            fy = m10 * nx + m11 * ny + m12 * nz
            fz = m20 * nx + m21 * ny + m22 * nz
            length = sqrt(fx * fx + fy * fy + fz * fz)
            if length > 1e-30:
                fx, fy, fz = fx / length, fy / length, fz / length
            else:
                fx = fy = fz = 0.0

        pack_into(out, offset, fx, fy, fz, pax, pay, paz,
                  pbx, pby, pbz, pcx, pcy, pcz, attr)
        offset += 50

        if pax < lo[0]: lo[0] = pax
        if pbx < lo[0]: lo[0] = pbx
        if pcx < lo[0]: lo[0] = pcx
        if pax > hi[0]: hi[0] = pax
        if pbx > hi[0]: hi[0] = pbx
        if pcx > hi[0]: hi[0] = pcx
        if pay < lo[1]: lo[1] = pay
        if pby < lo[1]: lo[1] = pby
        if pcy < lo[1]: lo[1] = pcy
        if pay > hi[1]: hi[1] = pay
        if pby > hi[1]: hi[1] = pby
        if pcy > hi[1]: hi[1] = pcy
        if paz < lo[2]: lo[2] = paz
        if pbz < lo[2]: lo[2] = pbz
        if pcz < lo[2]: lo[2] = pcz
        if paz > hi[2]: hi[2] = paz
        if pbz > hi[2]: hi[2] = pbz
        if pcz > hi[2]: hi[2] = pcz

    if mesh.count == 0:
        lo = hi = [0.0, 0.0, 0.0]
    return bytes(out), [tuple(lo), tuple(hi)]               # type: ignore[return-value]


# --------------------------------------------------------------------------- #
# URDF model
# --------------------------------------------------------------------------- #

@dataclass
class Inertial:
    mass: float
    origin: Transform
    inertia: Mat3          # about the COM, in the inertial-origin axes


@dataclass
class Visual:
    """A `<visual>` or `<collision>` element."""

    origin: Transform
    geometry: ET.Element   # verbatim <geometry>
    material: ET.Element | None
    name: str | None
    mesh_filename: str | None
    mesh_scale: Vec3


@dataclass
class Link:
    name: str
    inertial: Inertial | None = None
    visuals: list[Visual] = field(default_factory=list)
    collisions: list[Visual] = field(default_factory=list)
    element: ET.Element | None = None


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: Transform
    axis: Vec3 | None
    element: ET.Element | None = None   # kept so limits/dynamics/mimic survive verbatim


@dataclass
class Model:
    name: str
    links: dict[str, Link]
    joints: dict[str, Joint]
    extras: list[ET.Element] = field(default_factory=list)  # <material>, <gazebo>, ...


def _parse_origin(parent: ET.Element | None) -> Transform:
    if parent is None:
        return Transform()
    node = parent.find("origin")
    if node is None:
        return Transform()
    xyz = [float(v) for v in (node.get("xyz") or "0 0 0").split()]
    rpy = [float(v) for v in (node.get("rpy") or "0 0 0").split()]
    return Transform.from_xyz_rpy(xyz, rpy)


def _parse_visual(node: ET.Element) -> Visual:
    geometry = node.find("geometry")
    if geometry is None:
        geometry = ET.Element("geometry")
    mesh = geometry.find("mesh")
    filename = mesh.get("filename") if mesh is not None else None
    scale: Vec3 = (1.0, 1.0, 1.0)
    if mesh is not None and mesh.get("scale"):
        parts = [float(v) for v in mesh.get("scale", "1 1 1").split()]
        if len(parts) == 1:
            parts *= 3
        scale = (parts[0], parts[1], parts[2])
    return Visual(origin=_parse_origin(node), geometry=geometry,
                  material=node.find("material"), name=node.get("name"),
                  mesh_filename=filename, mesh_scale=scale)


def parse_urdf(path: str) -> Model:
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise ValueError(f"{path}: root element is <{root.tag}>, expected <robot>")

    links: dict[str, Link] = {}
    for node in root.findall("link"):
        name = node.get("name")
        if not name:
            raise ValueError("found a <link> without a name")
        link = Link(name=name, element=node)
        inertial_node = node.find("inertial")
        if inertial_node is not None:
            mass_node = inertial_node.find("mass")
            mass = float(mass_node.get("value", "0")) if mass_node is not None else 0.0
            tensor_node = inertial_node.find("inertia")
            if tensor_node is not None:
                ixx = float(tensor_node.get("ixx", "0")); ixy = float(tensor_node.get("ixy", "0"))
                ixz = float(tensor_node.get("ixz", "0")); iyy = float(tensor_node.get("iyy", "0"))
                iyz = float(tensor_node.get("iyz", "0")); izz = float(tensor_node.get("izz", "0"))
            else:
                ixx = ixy = ixz = iyy = iyz = izz = 0.0
            link.inertial = Inertial(mass, _parse_origin(inertial_node),
                                     ((ixx, ixy, ixz), (ixy, iyy, iyz), (ixz, iyz, izz)))
        link.visuals = [_parse_visual(v) for v in node.findall("visual")]
        link.collisions = [_parse_visual(c) for c in node.findall("collision")]
        if name in links:
            raise ValueError(f"duplicate link name {name!r}")
        links[name] = link

    joints: dict[str, Joint] = {}
    for node in root.findall("joint"):
        name = node.get("name")
        if not name:
            raise ValueError("found a <joint> without a name")
        parent_node, child_node = node.find("parent"), node.find("child")
        if parent_node is None or child_node is None:
            raise ValueError(f"joint {name!r} is missing <parent> or <child>")
        axis_node = node.find("axis")
        axis: Vec3 | None = None
        if axis_node is not None:
            vals = [float(v) for v in (axis_node.get("xyz") or "1 0 0").split()]
            axis = (vals[0], vals[1], vals[2])
        if name in joints:
            raise ValueError(f"duplicate joint name {name!r}")
        joints[name] = Joint(name=name, type=node.get("type", "fixed"),
                             parent=parent_node.get("link", ""),
                             child=child_node.get("link", ""),
                             origin=_parse_origin(node), axis=axis, element=node)

    extras = [n for n in root if n.tag not in ("link", "joint")]
    return Model(root.get("name", "robot"), links, joints, extras)


# --------------------------------------------------------------------------- #
# Kinematics helpers
# --------------------------------------------------------------------------- #

def child_map(model: Model) -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    for joint in model.joints.values():
        out.setdefault(joint.parent, []).append(joint.name)
    return out


def parent_joint_map(model: Model) -> dict[str, str]:
    out: dict[str, str] = {}
    for joint in model.joints.values():
        if joint.child in out:
            raise ValueError(
                f"link {joint.child!r} is the child of both {out[joint.child]!r} and "
                f"{joint.name!r}; URDF requires a tree")
        out[joint.child] = joint.name
    return out


def tree_roots(model: Model) -> list[str]:
    parents = parent_joint_map(model)
    return [name for name in model.links if name not in parents]


def forward_kinematics(model: Model) -> dict[str, Transform]:
    """World pose of every link with all joints at zero."""
    children = child_map(model)
    poses: dict[str, Transform] = {}
    for root in tree_roots(model):
        stack = [(root, Transform())]
        while stack:
            link, pose = stack.pop()
            if link in poses:
                raise ValueError(f"cycle detected in the joint graph at {link!r}")
            poses[link] = pose
            for joint_name in children.get(link, []):
                joint = model.joints[joint_name]
                stack.append((joint.child, pose @ joint.origin))
    missing = set(model.links) - set(poses)
    if missing:
        raise ValueError(f"links unreachable from any root: {sorted(missing)}")
    return poses


def link_depths(model: Model) -> dict[str, int]:
    children = child_map(model)
    depths: dict[str, int] = {}
    for root in tree_roots(model):
        stack = [(root, 0)]
        while stack:
            link, depth = stack.pop()
            depths[link] = depth
            for joint_name in children.get(link, []):
                stack.append((model.joints[joint_name].child, depth + 1))
    return depths


def connected_components(model: Model, ignore: set[str] = frozenset()) -> dict[str, str]:
    """Undirected connectivity over all joints except those named in `ignore`."""
    parent = {name: name for name in model.links}

    def find(x: str) -> str:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for joint in model.joints.values():
        if joint.name in ignore:
            continue
        a, b = find(joint.parent), find(joint.child)
        if a != b:
            parent[a] = b
    return {name: find(name) for name in model.links}


# --------------------------------------------------------------------------- #
# Inertia
# --------------------------------------------------------------------------- #

def inertial_in_frame(inertial: Inertial | None, xf: Transform) -> tuple[float, Vec3, Mat3]:
    """Express a link's inertial as (mass, COM, inertia about the COM) in `xf`'s frame."""
    if inertial is None or inertial.mass <= 0.0:
        return 0.0, ZERO3, ((0.0,) * 3,) * 3          # type: ignore[return-value]
    # <inertia> is given about the COM but in the axes of the <inertial> origin.
    tensor = congruence(inertial.origin.rot, inertial.inertia)
    total = xf @ inertial.origin
    return inertial.mass, total.pos, congruence(xf.rot, tensor)


def combine_inertials(parts: Sequence[tuple[float, Vec3, Mat3]]) -> tuple[float, Vec3, Mat3]:
    """Merge (mass, COM, inertia-about-COM) triples already in a common frame."""
    mass = sum(p[0] for p in parts)
    if mass <= 0.0:
        return 0.0, ZERO3, ((0.0,) * 3,) * 3          # type: ignore[return-value]
    com = tuple(sum(p[0] * p[1][i] for p in parts) / mass for i in range(3))
    tensor: Mat3 = ((0.0,) * 3,) * 3                  # type: ignore[assignment]
    for part_mass, part_com, part_tensor in parts:
        if part_mass <= 0.0:
            continue
        d = vec_sub(part_com, com)
        dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2]
        # Parallel axis: I + m (|d|^2 E - d d^T)
        shift = ((part_mass * (dd - d[0] * d[0]), part_mass * (-d[0] * d[1]), part_mass * (-d[0] * d[2])),
                 (part_mass * (-d[1] * d[0]), part_mass * (dd - d[1] * d[1]), part_mass * (-d[1] * d[2])),
                 (part_mass * (-d[2] * d[0]), part_mass * (-d[2] * d[1]), part_mass * (dd - d[2] * d[2])))
        tensor = mat_add(tensor, mat_add(part_tensor, shift))
    return mass, com, tensor                          # type: ignore[return-value]


def inertia_about_origin(mass: float, com: Vec3, tensor: Mat3) -> Mat3:
    """Shift an inertia from the COM to the frame origin (for global comparison)."""
    dd = com[0] ** 2 + com[1] ** 2 + com[2] ** 2
    shift = ((mass * (dd - com[0] * com[0]), mass * (-com[0] * com[1]), mass * (-com[0] * com[2])),
             (mass * (-com[1] * com[0]), mass * (dd - com[1] * com[1]), mass * (-com[1] * com[2])),
             (mass * (-com[2] * com[0]), mass * (-com[2] * com[1]), mass * (dd - com[2] * com[2])))
    return mat_add(tensor, shift)


def global_inertial(model: Model) -> tuple[float, Vec3, Mat3]:
    poses = forward_kinematics(model)
    parts = [inertial_in_frame(link.inertial, poses[name])
             for name, link in model.links.items()]
    return combine_inertials(parts)


# --------------------------------------------------------------------------- #
# Mesh path resolution
# --------------------------------------------------------------------------- #

def resolve_mesh_path(filename: str, urdf_path: str,
                      package_dirs: dict[str, str]) -> str | None:
    match = re.match(r"^(?:package|model)://([^/]+)/(.*)$", filename)
    if match:
        package, relative = match.group(1), match.group(2)
    elif filename.startswith("file://"):
        path = filename[7:]
        if os.name == "nt" and re.match(r"^/[A-Za-z]:", path):
            path = path[1:]
        return path if os.path.isfile(path) else None
    else:
        package, relative = None, filename

    if package and package in package_dirs:
        candidate = os.path.join(package_dirs[package], relative)
        if os.path.isfile(candidate):
            return candidate

    directory = os.path.dirname(os.path.abspath(urdf_path))
    if not package:
        candidate = os.path.join(directory, relative)
        if os.path.isfile(candidate):
            return candidate

    previous = None
    while directory and directory != previous:
        if package:
            candidate = os.path.join(directory, package, relative)
            if os.path.isfile(candidate):
                return candidate
        candidate = os.path.join(directory, relative)
        if os.path.isfile(candidate):
            return candidate
        previous, directory = directory, os.path.dirname(directory)
    return None


# --------------------------------------------------------------------------- #
# Pass 1 -- mate connectors
# --------------------------------------------------------------------------- #

@dataclass
class MateConnectorReport:
    detected: list[str] = field(default_factory=list)
    de_proxied: list[str] = field(default_factory=list)
    absorbed: list[str] = field(default_factory=list)
    created_joints: list[str] = field(default_factory=list)
    skipped_pairs: list[str] = field(default_factory=list)
    kept: list[str] = field(default_factory=list)


def detect_mate_connectors(model: Model, pattern: str, mass_threshold: float) -> list[str]:
    """A mate connector is a proxy frame: name matches, no geometry, negligible mass."""
    regex = re.compile(pattern)
    found = []
    for name, link in model.links.items():
        if not regex.search(name):
            continue
        if link.visuals or link.collisions:
            continue
        mass = link.inertial.mass if link.inertial else 0.0
        if mass > mass_threshold:
            continue
        found.append(name)
    return sorted(found)


def _joint_frames_match(a: Transform, b: Transform,
                        position_tol: float, rotation_tol: float) -> bool:
    if vec_norm(vec_sub(a.pos, b.pos)) > position_tol:
        return False
    return max(abs(a.rot[i][j] - b.rot[i][j]) for i in range(3) for j in range(3)) <= rotation_tol


def connect_coincident_mate_connectors(model: Model, connectors: list[str],
                                       position_tol: float, rotation_tol: float,
                                       report: MateConnectorReport) -> None:
    """Turn a coincident mate-connector pair into a direct joint between parents.

    Only fires when the two parent links are in genuinely disconnected components:
    repeated part instances (four identical servos, six identical horns) place
    their mate connectors at the same local spot, so raw coincidence is *not*
    evidence of a mate.  Pairs whose parents are already connected are reported
    and left alone -- adding a joint there would create a kinematic loop, which
    URDF cannot express.
    """
    poses = forward_kinematics(model)
    parents = parent_joint_map(model)
    attachments = {parents[c] for c in connectors if c in parents}

    for i, first in enumerate(connectors):
        for second in connectors[i + 1:]:
            if not _joint_frames_match(poses[first], poses[second], position_tol, rotation_tol):
                continue
            if first not in parents or second not in parents:
                continue
            link_a = model.joints[parents[first]].parent
            link_b = model.joints[parents[second]].parent
            if link_a == link_b:
                continue
            components = connected_components(model, ignore=attachments)
            if components[link_a] == components[link_b]:
                report.skipped_pairs.append(
                    f"{first}({link_a}) <-> {second}({link_b}): already connected")
                continue

            # T_a->b = T_a->mcA @ (T_b->mcB)^-1, with mcA and mcB coincident.
            to_a = model.joints[parents[first]].origin
            to_b = model.joints[parents[second]].origin
            name = f"mated_{link_a}_to_{link_b}"
            suffix = 0
            while name in model.joints:
                suffix += 1
                name = f"mated_{link_a}_to_{link_b}_{suffix}"

            # Whichever side is not already rooted becomes the child.
            depths = link_depths(model)
            if depths.get(link_a, 0) > depths.get(link_b, 0):
                link_a, link_b = link_b, link_a
                to_a, to_b = to_b, to_a
            model.joints[name] = Joint(name=name, type="fixed", parent=link_a,
                                       child=link_b, origin=to_a @ to_b.inverse(),
                                       axis=None, element=None)
            report.created_joints.append(f"{name}: {link_a} -> {link_b} (from {first}/{second})")
            parents = parent_joint_map(model)


def classify_mate_connectors(model: Model, connectors: list[str],
                             report: MateConnectorReport) -> None:
    """Record how the rigid-cluster pass will dispose of each proxy frame.

    Every mate-connector topology is resolved by the cluster pass, so nothing is
    rewired here.  Naming the four cases explicitly, with MC a mate connector:

      P --fixed--> MC                  MC joins P's rigid group and disappears;
                                       its mass is folded in.
      P --fixed--> MC --movable--> C   MC joins P's rigid group, so the movable
                                       joint is re-parented onto P directly with
                                       the transforms composed.  This is the
                                       proxy case: the joint ends up between the
                                       two real links, with no frame in between.
      P --movable--> MC --fixed--> C   MC and C form one rigid group.  The group
                                       *keeps MC's frame* and takes C's name.
      P --movable--> MC --movable--> C MC carries two DOF and is a real body; it
                                       has to survive as a link.

    The third case is why the proxy is not spliced out.  A URDF joint rotates
    about its axis through the child link's frame origin, so folding the fixed
    offset Oc into the joint origin would move the axis to a parallel line
    unless Oc's translation happens to lie on the axis: composing gives
    Oc^-1 Rot(a, t) Oc = (Rc^T Ra Rc, Rc^T (Ra - I) tc), whose translation
    term vanishes only when Ra tc == tc.  Keeping MC's frame and renaming the
    body is exact for every offset.
    """
    children = child_map(model)
    parents = parent_joint_map(model)

    for connector in connectors:
        outgoing = [model.joints[j] for j in children.get(connector, [])]
        incoming = model.joints[parents[connector]] if connector in parents else None
        upstream_movable = incoming is not None and incoming.type != "fixed"
        movable_children = [j for j in outgoing if j.type != "fixed"]

        if upstream_movable and movable_children:
            report.kept.append(
                f"{connector}: movable joints on both sides ({incoming.name}/"
                f"{movable_children[0].name}); it carries real degrees of freedom "
                f"and is kept as a link")
        elif incoming is not None and not upstream_movable and movable_children:
            report.de_proxied += [
                f"{j.name}: now joins {incoming.parent} -> {j.child} directly "
                f"(was proxied through {connector})" for j in movable_children]


# --------------------------------------------------------------------------- #
# Pass 2 -- rigid clusters
# --------------------------------------------------------------------------- #

@dataclass
class Cluster:
    name: str                       # name of the merged link
    representative: str             # link whose frame the merged link inherits
    members: dict[str, Transform]   # member link -> transform into the representative frame


def build_clusters(model: Model, keep_pattern: str | None,
                   proxies: set[str] = frozenset()) -> tuple[list[Cluster], dict[str, str]]:
    """Group links joined by fixed joints; the shallowest member keeps its frame.

    The frame has to come from the shallowest member so that the movable joint
    above the group still attaches to the link it names.  The *name*, though, is
    free: when the shallowest member is a mate-connector proxy the group takes
    the name of a real part instead, so no proxy name reaches the output.
    """
    keep = re.compile(keep_pattern) if keep_pattern else None
    merged_joints = {j.name for j in model.joints.values()
                     if j.type == "fixed" and not (keep and keep.search(j.name))}

    parent = {name: name for name in model.links}

    def find(x: str) -> str:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for joint in model.joints.values():
        if joint.name in merged_joints:
            a, b = find(joint.parent), find(joint.child)
            if a != b:
                parent[a] = b

    groups: dict[str, list[str]] = {}
    for name in model.links:
        groups.setdefault(find(name), []).append(name)

    depths = link_depths(model)
    parents = parent_joint_map(model)
    clusters: list[Cluster] = []
    for members in groups.values():
        representative = min(members, key=lambda n: (depths.get(n, 1 << 30), n))
        transforms = {representative: Transform()}
        # Walk outward from the representative along the merged fixed joints.
        pending = [representative]
        while pending:
            current = pending.pop()
            for joint in model.joints.values():
                if joint.name not in merged_joints:
                    continue
                if joint.parent == current and joint.child not in transforms:
                    transforms[joint.child] = transforms[current] @ joint.origin
                    pending.append(joint.child)
                elif joint.child == current and joint.parent not in transforms:
                    transforms[joint.parent] = transforms[current] @ joint.origin.inverse()
                    pending.append(joint.parent)
        missing = set(members) - set(transforms)
        if missing:
            raise ValueError(f"cluster around {representative!r} is not fixed-connected: {missing}")

        name = representative
        if representative in proxies:
            real = [m for m in members if m not in proxies]
            if real:
                name = min(real, key=lambda n: (depths.get(n, 1 << 30), n))
        clusters.append(Cluster(name=name, representative=representative,
                                members=transforms))

    # Sanity: no movable joint may point at a non-representative link, or the
    # merged link frame would not be the one the joint attaches to.
    lookup = {member: cluster for cluster in clusters for member in cluster.members}
    for joint in model.joints.values():
        if joint.name in merged_joints:
            continue
        cluster = lookup[joint.child]
        if joint.child != cluster.representative:
            raise ValueError(
                f"joint {joint.name!r} drives {joint.child!r}, which is not the frame "
                f"origin of its rigid group (that is {cluster.representative!r}); "
                f"the input is not a tree of rigid bodies")
    return clusters, {member: c.name for c in clusters for member in c.members}


def apply_name_map(clusters: list[Cluster], name_map: dict[str, str],
                   model: Model) -> list[str]:
    """Rename subassemblies.  Keys are `_`-prefixed comments, or one of:

      "joint:<joint name>"  the body driven by that joint, i.e. the group its
                            child link belongs to.  Prefer this: CAD exporters
                            renumber part instances between exports, so a group
                            that was `servo_horn_4hole_4` can silently become
                            `servo_horn_4hole_1` and a link-keyed map would then
                            put the wrong name on the wrong body.
      "<link name>"         the group whose representative is that link, for
                            bodies no joint drives (the root).

    The prefix is required because a URDF may use the same name for a link and
    a joint -- Onshape emits exactly that for mate connectors.
    """
    by_representative = {c.representative: c for c in clusters}
    used: set[str] = set()
    warnings: list[str] = []

    for key, value in name_map.items():
        if key.startswith("_"):
            continue
        if key.startswith("joint:"):
            joint_name = key[len("joint:"):]
            joint = model.joints.get(joint_name)
            if joint is None:
                warnings.append(f"--name-map key {key!r} names no joint")
                continue
            cluster = by_representative.get(joint.child)
            if cluster is None:
                warnings.append(f"--name-map key {key!r} drives {joint.child!r}, "
                                f"which is not a subassembly frame")
                continue
        else:
            cluster = by_representative.get(key)
            if cluster is None:
                warnings.append(f"--name-map key {key!r} matches no subassembly "
                                f"representative (for a driven body use 'joint:<name>')")
                continue
        cluster.name = value
        used.add(key)
    seen: dict[str, str] = {}
    for cluster in clusters:
        if cluster.name in seen:
            raise ValueError(f"--name-map gives both {seen[cluster.name]!r} and "
                             f"{cluster.representative!r} the name {cluster.name!r}")
        seen[cluster.name] = cluster.representative
    return warnings


# --------------------------------------------------------------------------- #
# Pass 3 -- build the merged model
# --------------------------------------------------------------------------- #

def _element(tag: str, **attrs: str) -> ET.Element:
    node = ET.Element(tag)
    for key, value in attrs.items():
        node.set(key, value)
    return node


def _fmt(value: float, precision: int) -> str:
    if abs(value) < 1e-15:
        return "0"
    text = f"{value:.{precision}g}"
    return text


def _origin_element(xf: Transform, precision: int) -> ET.Element:
    xyz, rpy = xf.to_xyz_rpy()
    return _element("origin",
                    xyz=" ".join(_fmt(v, precision) for v in xyz),
                    rpy=" ".join(_fmt(v, precision) for v in rpy))


def build_merged_model(model: Model, clusters: list[Cluster], link_of: dict[str, str],
                       keep_pattern: str | None) -> Model:
    keep = re.compile(keep_pattern) if keep_pattern else None
    merged = Model(name=model.name, links={}, joints={}, extras=model.extras)

    for cluster in clusters:
        link = Link(name=cluster.name)
        parts = [inertial_in_frame(model.links[member].inertial, xf)
                 for member, xf in cluster.members.items()]
        mass, com, tensor = combine_inertials(parts)
        if mass > 0.0:
            link.inertial = Inertial(mass, Transform(IDENTITY3, com), tensor)
        for member, xf in cluster.members.items():
            source = model.links[member]
            for visual in source.visuals:
                link.visuals.append(Visual(origin=xf @ visual.origin, geometry=visual.geometry,
                                           material=visual.material,
                                           name=visual.name or member,
                                           mesh_filename=visual.mesh_filename,
                                           mesh_scale=visual.mesh_scale))
            for collision in source.collisions:
                link.collisions.append(Visual(origin=xf @ collision.origin,
                                              geometry=collision.geometry,
                                              material=None, name=collision.name or member,
                                              mesh_filename=collision.mesh_filename,
                                              mesh_scale=collision.mesh_scale))
        merged.links[cluster.name] = link

    for joint in model.joints.values():
        if joint.type == "fixed" and not (keep and keep.search(joint.name)):
            continue                                   # absorbed into a cluster
        parent_cluster = next(c for c in clusters if joint.parent in c.members)
        new_origin = parent_cluster.members[joint.parent] @ joint.origin
        merged.joints[joint.name] = Joint(
            name=joint.name, type=joint.type,
            parent=link_of[joint.parent], child=link_of[joint.child],
            origin=new_origin,
            # The axis is expressed in the child frame, which merging preserves.
            axis=joint.axis, element=joint.element)
    return merged


def fuse_cluster_meshes(model: Model, merged: Model, clusters: list[Cluster],
                        urdf_path: str, mesh_dir: str, package: str,
                        package_dirs: dict[str, str], collision: bool,
                        verbose: bool) -> tuple[dict[str, dict], list[str]]:
    """Bake each member STL into the subassembly frame and concatenate them."""
    os.makedirs(mesh_dir, exist_ok=True)
    cache: dict[str, RawMesh] = {}
    stats: dict[str, dict] = {}
    warnings: list[str] = []

    for cluster in clusters:
        link = merged.links[cluster.name]
        fusable = [v for v in link.visuals if v.mesh_filename]
        other = [v for v in link.visuals if not v.mesh_filename]
        if not fusable:
            continue

        chunks: list[bytes] = []
        total = 0
        bounds_lo = [math.inf] * 3
        bounds_hi = [-math.inf] * 3
        sources: list[dict] = []
        best_material: ET.Element | None = None
        best_count = -1

        for visual in fusable:
            path = resolve_mesh_path(visual.mesh_filename or "", urdf_path, package_dirs)
            if path is None:
                warnings.append(f"{cluster.name}: cannot resolve mesh {visual.mesh_filename!r}; "
                                f"kept as a separate <visual>")
                other.append(visual)
                continue
            if path not in cache:
                cache[path] = read_stl(path)
            mesh = cache[path]
            payload, (lo, hi) = transform_mesh(mesh, visual.origin, visual.mesh_scale)
            chunks.append(payload)
            total += mesh.count
            for i in range(3):
                bounds_lo[i] = min(bounds_lo[i], lo[i])
                bounds_hi[i] = max(bounds_hi[i], hi[i])
            sources.append({"link": visual.name, "mesh": os.path.basename(path),
                            "triangles": mesh.count,
                            "bounds_in_subassembly": [list(lo), list(hi)]})
            if mesh.count > best_count:
                best_count, best_material = mesh.count, visual.material
            if verbose:
                print(f"    {visual.name:<24s} {mesh.count:>7d} tri  <- {os.path.basename(path)}",
                      file=sys.stderr)

        if not chunks:
            continue

        filename = f"{cluster.name}.stl"
        out_path = os.path.join(mesh_dir, filename)
        write_stl(out_path, f"{cluster.name}. Fused subassembly, {len(sources)} parts. "
                            f"Units = meters", chunks, total)

        fused = Visual(origin=Transform(), geometry=ET.fromstring(
            f'<geometry><mesh filename="package://{package}/meshes/{filename}" '
            f'scale="1 1 1" /></geometry>'), material=best_material,
            name=cluster.name, mesh_filename=f"package://{package}/meshes/{filename}",
            mesh_scale=(1.0, 1.0, 1.0))
        link.visuals = [fused] + other
        if collision:
            link.collisions = [Visual(origin=Transform(), geometry=copy.deepcopy(fused.geometry),
                                      material=None, name=cluster.name,
                                      mesh_filename=fused.mesh_filename,
                                      mesh_scale=(1.0, 1.0, 1.0))]
        stats[cluster.name] = {"file": filename, "triangles": total,
                               "bytes": os.path.getsize(out_path),
                               "bounds_in_link": [bounds_lo, bounds_hi],
                               "sources": sources}
    return stats, warnings


def serialize(merged: Model, robot_name: str, precision: int) -> ET.ElementTree:
    root = _element("robot", name=robot_name)
    for extra in merged.extras:
        root.append(copy.deepcopy(extra))

    for link in merged.links.values():
        node = _element("link", name=link.name)
        if link.inertial is not None:
            inertial = ET.SubElement(node, "inertial")
            inertial.append(_origin_element(link.inertial.origin, precision))
            ET.SubElement(inertial, "mass").set("value", _fmt(link.inertial.mass, precision))
            tensor = link.inertial.inertia
            ET.SubElement(inertial, "inertia", {
                "ixx": _fmt(tensor[0][0], precision), "ixy": _fmt(tensor[0][1], precision),
                "ixz": _fmt(tensor[0][2], precision), "iyy": _fmt(tensor[1][1], precision),
                "iyz": _fmt(tensor[1][2], precision), "izz": _fmt(tensor[2][2], precision)})
        for kind, items in (("visual", link.visuals), ("collision", link.collisions)):
            for item in items:
                child = ET.SubElement(node, kind)
                if item.name and len(items) > 1:
                    child.set("name", item.name)
                child.append(_origin_element(item.origin, precision))
                child.append(copy.deepcopy(item.geometry))
                if kind == "visual" and item.material is not None:
                    child.append(copy.deepcopy(item.material))
        root.append(node)

    for joint in merged.joints.values():
        node = _element("joint", name=joint.name, type=joint.type)
        node.append(_origin_element(joint.origin, precision))
        if joint.axis is not None:
            node.append(_element("axis", xyz=" ".join(_fmt(v, precision) for v in joint.axis)))
        node.append(_element("parent", link=joint.parent))
        node.append(_element("child", link=joint.child))
        if joint.element is not None:
            # Carry limits/dynamics/mimic/safety/calibration across untouched.
            for extra in joint.element:
                if extra.tag not in ("origin", "axis", "parent", "child"):
                    node.append(copy.deepcopy(extra))
        root.append(node)

    ET.indent(root, space="    ")
    return ET.ElementTree(root)


def write_manifest(out_dir: str, urdf_path: str, robot_name: str, package: str,
                   original: Model, merged: Model, clusters: list[Cluster],
                   link_of: dict[str, str], report: MateConnectorReport,
                   stats: dict[str, dict]) -> str:
    """Record enough to migrate anything that referenced the old link frames."""
    manifest = {
        "source_urdf": urdf_path,
        "robot_name": robot_name,
        "package": package,
        "links_before": len(original.links),
        "links_after": len(merged.links),
        "joints_before": len(original.joints),
        "joints_after": len(merged.joints),
        "mate_connectors": {
            "detected": report.detected,
            "folded_into_a_real_body": [c for c in report.absorbed
                                        if not any(c in k for k in report.kept)],
            "joints_de_proxied": report.de_proxied,
            "created_joints": report.created_joints,
            "coincident_but_already_connected": report.skipped_pairs,
            "kept_as_links": report.kept,
        },
        "subassemblies": {},
        "link_map": link_of,
        "meshes": stats,
    }
    for cluster in clusters:
        link = merged.links[cluster.name]
        entry: dict = {"representative": cluster.representative, "members": {}}
        for member, xf in sorted(cluster.members.items()):
            xyz, rpy = xf.to_xyz_rpy()
            entry["members"][member] = {"xyz": list(xyz), "rpy": list(rpy)}
        if link.inertial is not None:
            entry["mass"] = link.inertial.mass
            entry["com"] = list(link.inertial.origin.pos)
            entry["inertia"] = [list(row) for row in link.inertial.inertia]
        manifest["subassemblies"][cluster.name] = entry

    manifest_path = os.path.join(out_dir, f"{robot_name}_manifest.json")
    with open(manifest_path, "w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2)
    return manifest_path


# --------------------------------------------------------------------------- #
# Verification
# --------------------------------------------------------------------------- #

def verify(original: Model, merged: Model, clusters: list[Cluster],
           link_of: dict[str, str], tolerance: float) -> list[str]:
    """Re-derive world-frame quantities from both models and compare."""
    problems: list[str] = []
    before = forward_kinematics(original)
    after = forward_kinematics(merged)

    # Global mass properties about the world origin.
    mass_a, com_a, tensor_a = global_inertial(original)
    mass_b, com_b, tensor_b = global_inertial(merged)
    if abs(mass_a - mass_b) > tolerance * max(1.0, abs(mass_a)):
        problems.append(f"total mass changed: {mass_a!r} -> {mass_b!r}")
    if mass_a > 0 and vec_norm(vec_sub(com_a, com_b)) > tolerance * max(1.0, vec_norm(com_a)):
        problems.append(f"centre of mass moved: {com_a} -> {com_b}")
    world_a = inertia_about_origin(mass_a, com_a, tensor_a)
    world_b = inertia_about_origin(mass_b, com_b, tensor_b)
    scale = max(abs(world_a[i][j]) for i in range(3) for j in range(3)) or 1.0
    for i in range(3):
        for j in range(3):
            if abs(world_a[i][j] - world_b[i][j]) > tolerance * scale:
                problems.append(
                    f"inertia about the world origin changed at [{i}][{j}]: "
                    f"{world_a[i][j]!r} -> {world_b[i][j]!r}")

    # Every movable joint must survive with the same world pose, axis and limits.
    movable_before = {n: j for n, j in original.joints.items() if j.type != "fixed"}
    movable_after = {n: j for n, j in merged.joints.items() if j.type != "fixed"}
    if set(movable_before) != set(movable_after):
        lost = sorted(set(movable_before) - set(movable_after))
        gained = sorted(set(movable_after) - set(movable_before))
        if lost:
            problems.append(f"movable joints lost: {lost}")
        if gained:
            problems.append(f"movable joints appeared: {gained}")

    for name in sorted(set(movable_before) & set(movable_after)):
        a, b = movable_before[name], movable_after[name]
        if a.type != b.type:
            problems.append(f"joint {name}: type {a.type} -> {b.type}")
        frame_a = before[a.parent] @ a.origin
        frame_b = after[b.parent] @ b.origin
        if vec_norm(vec_sub(frame_a.pos, frame_b.pos)) > tolerance:
            problems.append(f"joint {name}: world position moved by "
                            f"{vec_norm(vec_sub(frame_a.pos, frame_b.pos)):.3e} m")
        rot_error = max(abs(frame_a.rot[i][j] - frame_b.rot[i][j])
                        for i in range(3) for j in range(3))
        if rot_error > tolerance:
            problems.append(f"joint {name}: world orientation changed by {rot_error:.3e}")
        if a.axis is not None and b.axis is not None:
            axis_a = before[a.child].rotate(a.axis)
            axis_b = after[b.child].rotate(b.axis)
            if vec_norm(vec_sub(axis_a, axis_b)) > tolerance * max(1.0, vec_norm(axis_a)):
                problems.append(f"joint {name}: world axis moved {axis_a} -> {axis_b}")
        limit_a = a.element.find("limit") if a.element is not None else None
        limit_b = b.element.find("limit") if b.element is not None else None
        if (limit_a is None) != (limit_b is None):
            problems.append(f"joint {name}: <limit> presence changed")
        elif limit_a is not None and limit_b is not None and limit_a.attrib != limit_b.attrib:
            problems.append(f"joint {name}: limits {limit_a.attrib} -> {limit_b.attrib}")

    # Every source visual must land in the same world pose.
    baked = {c.name: c for c in clusters}
    for name, link in original.links.items():
        target = merged.links[link_of[name]]
        cluster = baked[link_of[name]]
        for index, visual in enumerate(link.visuals):
            expected = before[name] @ visual.origin
            actual = after[target.name] @ (cluster.members[name] @ visual.origin)
            if vec_norm(vec_sub(expected.pos, actual.pos)) > tolerance:
                problems.append(f"{name} visual[{index}]: world position moved by "
                                f"{vec_norm(vec_sub(expected.pos, actual.pos)):.3e} m")
            rot_error = max(abs(expected.rot[i][j] - actual.rot[i][j])
                            for i in range(3) for j in range(3))
            if rot_error > tolerance:
                problems.append(f"{name} visual[{index}]: world orientation changed "
                                f"by {rot_error:.3e}")
    return problems


def verify_meshes(stats: dict[str, dict], merged: Model, mesh_dir: str,
                  tolerance: float) -> list[str]:
    """Reload the written STLs and check triangle counts and bounds.

    `tolerance` is deliberately looser than the pose tolerance: STL stores
    coordinates as float32, so a value computed in float64 and read back is off
    by ~1e-8 at this model's scale.  The default is still three orders of
    magnitude finer than any geometry error worth catching.
    """
    problems: list[str] = []
    for link_name, info in stats.items():
        path = os.path.join(mesh_dir, info["file"])
        mesh = read_stl(path)
        expected = sum(s["triangles"] for s in info["sources"])
        if mesh.count != expected:
            problems.append(f"{info['file']}: {mesh.count} triangles, expected {expected}")
        lo = [math.inf] * 3
        hi = [-math.inf] * 3
        for rec in _TRI.iter_unpack(mesh.payload):
            for offset in (3, 6, 9):
                for axis in range(3):
                    value = rec[offset + axis]
                    if value < lo[axis]:
                        lo[axis] = value
                    if value > hi[axis]:
                        hi[axis] = value
        want_lo, want_hi = info["bounds_in_link"]
        for axis in range(3):
            if abs(lo[axis] - want_lo[axis]) > tolerance or abs(hi[axis] - want_hi[axis]) > tolerance:
                problems.append(
                    f"{info['file']}: bounds on axis {axis} are "
                    f"[{lo[axis]:.6f}, {hi[axis]:.6f}], expected "
                    f"[{want_lo[axis]:.6f}, {want_hi[axis]:.6f}]")
    return problems


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("urdf", help="input URDF (as exported by Onshape)")
    parser.add_argument("-o", "--out-dir",
                        help="output package directory (default: the input package "
                             "next to it, with a trailing _full replaced by _sim)")
    parser.add_argument("--package", help="package name used in package:// mesh URIs "
                                          "(default: the output directory name)")
    parser.add_argument("--robot-name", help="robot name in the output URDF, and its "
                                             "file name (default: the package name)")
    parser.add_argument("--package-dir", action="append", default=[], metavar="NAME=PATH",
                        help="explicit location of a ROS package, repeatable")
    parser.add_argument("--name-map", metavar="JSON",
                        help="JSON file naming subassemblies; keys are 'joint:<name>' "
                             "(the body that joint drives, stable across re-exports) "
                             "or a representative link name")
    parser.add_argument("--mate-connector-pattern", default=r"mate_?connector",
                        help="regex identifying mate-connector proxy links "
                             "(default: %(default)s)")
    parser.add_argument("--mate-connector-max-mass", type=float, default=1e-6,
                        help="links above this mass are never treated as proxies "
                             "(default: %(default)s kg)")
    parser.add_argument("--mate-pair-position-tol", type=float, default=1e-5,
                        help="position tolerance for pairing mate connectors (default: %(default)s m)")
    parser.add_argument("--mate-pair-rotation-tol", type=float, default=1e-3,
                        help="orientation tolerance for pairing mate connectors (default: %(default)s)")
    parser.add_argument("--keep-fixed", metavar="REGEX",
                        help="regex of fixed joints to preserve instead of merging "
                             "(e.g. tool or sensor frames)")
    parser.add_argument("--no-fuse", action="store_true",
                        help="keep one <visual> per source part instead of fusing the STLs "
                             "(preserves per-part colours)")
    parser.add_argument("--collision", action="store_true",
                        help="also emit <collision> using the fused mesh")
    parser.add_argument("--manifest", action="store_true",
                        help="also write <robot name>_manifest.json, recording which source "
                             "links went into each subassembly and their transforms")
    parser.add_argument("--precision", type=int, default=12,
                        help="significant digits for numbers in the output (default: %(default)s)")
    parser.add_argument("--tolerance", type=float, default=1e-9,
                        help="verification tolerance for poses and mass properties "
                             "(default: %(default)s)")
    parser.add_argument("--mesh-tolerance", type=float, default=1e-6,
                        help="verification tolerance for fused mesh bounds; looser than "
                             "--tolerance because STL stores float32 (default: %(default)s m)")
    parser.add_argument("--no-verify", action="store_true", help="skip the verification pass")
    parser.add_argument("--no-verify-meshes", action="store_true",
                        help="skip reloading the written STLs")
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    urdf_path = os.path.abspath(args.urdf)
    if not os.path.isfile(urdf_path):
        print(f"error: {args.urdf}: no such file", file=sys.stderr)
        return 2

    package_dirs: dict[str, str] = {}
    for entry in args.package_dir:
        if "=" not in entry:
            print(f"error: --package-dir expects NAME=PATH, got {entry!r}", file=sys.stderr)
            return 2
        name, path = entry.split("=", 1)
        package_dirs[name] = path

    source_package = os.path.dirname(os.path.dirname(urdf_path))
    if args.out_dir:
        out_dir = os.path.abspath(args.out_dir)
    elif source_package.endswith("_full"):
        out_dir = source_package[:-len("_full")] + "_sim"
    else:
        out_dir = f"{source_package}_sim"
    package = args.package or os.path.basename(out_dir)

    model = parse_urdf(urdf_path)
    robot_name = args.robot_name or package
    original = parse_urdf(urdf_path)           # pristine copy for verification

    print(f"{os.path.basename(urdf_path)}: {len(model.links)} links, "
          f"{len(model.joints)} joints", file=sys.stderr)

    # Pass 1 -- mate connectors.
    report = MateConnectorReport()
    report.detected = detect_mate_connectors(
        model, args.mate_connector_pattern, args.mate_connector_max_mass)
    if report.detected:
        connect_coincident_mate_connectors(
            model, report.detected, args.mate_pair_position_tol,
            args.mate_pair_rotation_tol, report)
        classify_mate_connectors(model, report.detected, report)
    report.absorbed = [c for c in report.detected if c in model.links]

    print(f"mate connectors: {len(report.detected)} found, "
          f"{len(report.absorbed) - len(report.kept)} folded into a real body, "
          f"{len(report.de_proxied)} joints de-proxied, "
          f"{len(report.created_joints)} joints created", file=sys.stderr)
    for line in report.de_proxied:
        print(f"  = {line}", file=sys.stderr)
    for line in report.created_joints:
        print(f"  + {line}", file=sys.stderr)
    for line in report.kept:
        print(f"  ! {line}", file=sys.stderr)
    if args.verbose:
        for line in report.skipped_pairs:
            print(f"  . coincident but not mated -- {line}", file=sys.stderr)

    # Pass 2 -- rigid clusters.
    clusters, link_of = build_clusters(model, args.keep_fixed, set(report.detected))
    if args.name_map:
        with open(args.name_map, "r", encoding="utf-8") as handle:
            for warning in apply_name_map(clusters, json.load(handle), model):
                print(f"  ! {warning}", file=sys.stderr)
        link_of = {member: c.name for c in clusters for member in c.members}
    clusters.sort(key=lambda c: c.name)

    print(f"rigid subassemblies: {len(clusters)}", file=sys.stderr)
    for cluster in clusters:
        others = sorted(m for m in cluster.members if m != cluster.name)
        frame = ("" if cluster.representative == cluster.name
                 else f" [keeps the frame of {cluster.representative}]")
        print(f"  {cluster.name} <- {len(cluster.members)} parts{frame}"
              + (f": {', '.join(others)}" if others else ""), file=sys.stderr)

    # Pass 3 -- rebuild.
    merged = build_merged_model(model, clusters, link_of, args.keep_fixed)

    os.makedirs(os.path.join(out_dir, "urdf"), exist_ok=True)
    mesh_dir = os.path.join(out_dir, "meshes")
    stats: dict[str, dict] = {}
    if not args.no_fuse:
        print("fusing meshes...", file=sys.stderr)
        stats, mesh_warnings = fuse_cluster_meshes(
            model, merged, clusters, urdf_path, mesh_dir, package,
            package_dirs, args.collision, args.verbose)
        for warning in mesh_warnings:
            print(f"  ! {warning}", file=sys.stderr)
        for name in sorted(stats):
            info = stats[name]
            print(f"  {info['file']:<32s} {info['triangles']:>8d} tri  "
                  f"{info['bytes'] / 1e6:>6.2f} MB  ({len(info['sources'])} parts)",
                  file=sys.stderr)

    urdf_out = os.path.join(out_dir, "urdf", f"{robot_name}.urdf")
    tree = serialize(merged, robot_name, args.precision)
    with open(urdf_out, "wb") as handle:
        handle.write(b"<?xml version=\"1.0\" ?>\n")
        handle.write(f"<!-- Generated by {os.path.basename(__file__)} from "
                     f"{os.path.basename(urdf_path)}. Do not edit by hand. -->\n"
                     .encode("utf-8"))
        tree.write(handle, encoding="utf-8", xml_declaration=False)
        handle.write(b"\n")

    print(f"\nwrote {urdf_out}", file=sys.stderr)
    if args.manifest:
        manifest_path = write_manifest(out_dir, urdf_path, robot_name, package, original,
                                       merged, clusters, link_of, report, stats)
        print(f"      {manifest_path}", file=sys.stderr)
    print(f"{len(original.links)} links / {len(original.joints)} joints  ->  "
          f"{len(merged.links)} links / {len(merged.joints)} joints", file=sys.stderr)

    failures: list[str] = []
    if not args.no_verify:
        failures += verify(original, merged, clusters, link_of, args.tolerance)
        if stats and not args.no_verify_meshes:
            failures += verify_meshes(stats, merged, mesh_dir, args.mesh_tolerance)
    if failures:
        print(f"\nVERIFICATION FAILED ({len(failures)} problems):", file=sys.stderr)
        for problem in failures:
            print(f"  - {problem}", file=sys.stderr)
        return 1
    if not args.no_verify:
        print("verification passed: mass, centre of mass and inertia about the world "
              "origin, every joint pose/axis/limit, and every visual pose are unchanged",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
