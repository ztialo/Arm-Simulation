# Author: Zi Tao Li
# Date April 30th, 2025


import argparse
import sys
from pathlib import Path
sys.path.append("/home/zdli/Arm-Simulation/arm/src")

from isaaclab.app import AppLauncher
# create argparser
parser = argparse.ArgumentParser(description="Tutorial on spawning prims into the scene.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""
import piper_workStation
import task_setUp
import isaacsim.core.utils.prims as prim_utils
import omni.kit.app
import carb.input
from pxr import UsdShade, Sdf

import isaaclab.sim as sim_utils
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR



def main():
    """Main function."""
    print("Name of the task: ")
    task_folder = sys.stdin.readline().strip()

    ROOT = Path(__file__).resolve().parents[1]
    TASKS_DIR = (ROOT / "tasks").resolve()
    task_folder = Path(task_folder).expanduser().resolve()

    if task_folder.is_relative_to(TASKS_DIR):
        raise SystemExit(f"[!] {task_folder} is not inside {TASKS_DIR}")

    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    
    # Set main camera
    sim.set_camera_view([2.0, 0.0, 2.5], [-0.5, 0.0, 0.5])

    # initialize desk position
    desk1_pose = ((0.0, 0.0, 0.01), (0.0, 0.0, 90.0))

    desk_folder_path = (Path(__file__).resolve().parents[1] / "asset" / "workDesk_v1")
    piper_usd_path = (Path(__file__).resolve().parents[1] / "piper_description"/ "urdf"/ "piper_description_v100_camera"/ "piper_description_v100_camera.usd")
    task_folder_path = (Path(__file__).resolve().parents[1] / "tasks" / f"{task_folder}")

    # Calling WorkStationSetUp to set up the scene
    scene = piper_workStation.WorkStationSetUp(
        desk_folder_path = str(desk_folder_path),
        desk_pose = desk1_pose,
        desk_dynamic=False,
        desk_mass=40.0,
        piper_usd = str(piper_usd_path)
    )

    scene.build()

    # change the task folder path base on the task that you want to run
    task = task_setUp.TaskSetUp(
        task_folder_path = str(task_folder_path), 
        desk_pose = desk1_pose,
        desktop_height = desk1_pose[0][2] + 0.81  # 0.86 <- approximate desktop height
    )
    task.build()


    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")

    # keyboard input reset handler
    app = omni.kit.app.get_app()
    input = carb.input.acquire_input_interface()

    def on_update(e):
        if input.is_key_down(carb.input.KeyboardInput.NUMPAD_1):
            print("NumPad 1 pressed -> resetting simulation")
            task.build()
            sim.reset()

    app.get_update_event_stream().create_subscription_to_pop(on_update, name="key-check")

    # Simulate physics
    while simulation_app.is_running():
        # perform step
        sim.step()



if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
