import sys, os
from pathlib import Path
import subprocess

import shutil

if __name__ == "__main__":

    src_path = Path("/root/deep_grasp_pybullet/meshes/adversarial")
    files = list(src_path.glob("*.obj"))

    dest_path = Path("/root/deep_grasp_pybullet/meshes/adversarial/vhacd")

    vhadc_exe = "./TestVHACD"

    for f in files:
        f_path = str(f)
        f_stem = f.stem

        args = [vhadc_exe, f_path, "-h", "32"]
        subprocess.call(args)

        shutil.copyfile("./decomp.obj", str(dest_path / (f.stem + "_vhacd.obj")))
        shutil.copyfile("./decomp.stl", str(dest_path / (f.stem + "_vhacd.stl")))

    os.remove("./decomp.obj")
    os.remove("./decomp.stl")
    os.remove("./decomp.mtl")

