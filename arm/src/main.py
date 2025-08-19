# Author: Zi Tao Li
# Date April 30th, 2025


import argparse
import sys
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

import isaaclab.sim as sim_utils
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR


# def setup_scene():
#     # Add physics
#     stage = omni.usd.get_context().get_stage()
#     # Add a physics scene prim to stage
#     # scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
#     # #Set gravity vector
#     # scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
#     # scene.CreateGravityMagnitudeAttr().Set(981.0)

#     """Workspace set up with ground plane, light, and table"""
#     #Ground-plane
#     cfg_ground = sim_utils.GroundPlaneCfg()
#     cfg_ground.func("/World/defaultGroundPlane", cfg_ground)

#     #spawn distant light
#     cfg_light_distant = sim_utils.DistantLightCfg(
#         intensity=3000.0,
#         color=(0.75,0.75,0.75),
#     )
#     cfg_light_distant.func("/World/lightDistant", cfg_light_distant, translation=(1,0,10))

#     #spawn a workdesk next from converted USD file
#     desk_cfg = sim_utils.UsdFileCfg(usd_path="/home/zdli/Arm-Simulation/arm/objects/workDesk_new.usd") # change this path base on ur machine accordingly
#     desk_cfg.func("/World/Workspace/Desk", desk_cfg, translation=(0.0, 0.0, 0.65)) # 0.721-0.65 = 

#     #adding physics to workdesk
#     desk_prim = stage.GetPrimAtPath("/World/Workspace/Desk")
#     utils.setRigidBody(desk_prim, "convexDecomposition", True)
#     # UsdPhysics.RigidBodyAPI.Apply(desk_prim)
#     UsdPhysics.CollisionAPI.Apply(desk_prim)
#     UsdPhysics.MassAPI.Apply(desk_prim)

#     # CHANGE piper arm into URDF 
#     #spawn two piper arm in scene
#     leftArm_cfg = sim_utils.UsdFileCfg(usd_path="/home/zdli/Arm-Simulation/arm/piper_description/urdf/piper_description/piper_description.usd")
#     leftArm_cfg.func("/World/Workspace/LeftArm", leftArm_cfg, translation=(-0.25, -0.5, 0.794))

#     rightArm_cfg = sim_utils.UsdFileCfg(usd_path="/home/zdli/Arm-Simulation/arm/piper_description/urdf/piper_description/piper_description.usd")
#     rightArm_cfg.func("/World/Workspace/RightArm", rightArm_cfg, translation=(-0.25, 0.11, 0.794))



def main():
    """Main function."""

    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    
    # Set main camera
    sim.set_camera_view([2.0, 0.0, 2.5], [-0.5, 0.0, 0.5])

    # initialize desk position
    desk1_pose = ((0.0, 0.0, 0.01), (0.0, 0.0, 90.0))

    # Calling WorkStationSetUp to set up the scene
    scene = piper_workStation.WorkStationSetUp(
        desk_folder_path = "/home/zdli/Arm-Simulation/arm/asset/workDesk_v1",
        desk_pose = desk1_pose,
        desk_dynamic=False,
        desk_mass=40.0,
        piper_usd = "/home/zdli/Arm-Simulation/arm/piper_description/urdf/piper_description_v100_camera.usd"
    )

    scene.build()

    # change the task folder path base on the task that you want to run
    task = task_setUp.TaskSetUp(
        task_folder_path = "/home/zdli/Arm-Simulation/arm/tasks/Pickup_Pens", 
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
