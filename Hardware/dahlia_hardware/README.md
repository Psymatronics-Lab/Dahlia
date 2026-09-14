# Dahlia Arm Hardware
All the `.stl` files needed to print a Dahlia arm are included in this section of the repository, alongside `.urdf` files and meshes for simulation and RL training.

The full URDF contains a few unnecessary joints and sections that may hamper simulation performance, and thus a simulation-ready version with merged meshes and proper joints can be generated using the following code. ONly `numpy` needs to be installed for this.
```bash
python urdf_merge_subassemblies.py path/to/robot_full/urdf/robot_full.urdf -o path/to/robot_sim
```