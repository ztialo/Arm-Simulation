import math
import omni
import carb
import argparse
from pathlib import Path

import numpy as np
import omni.kit.app, omni.usd
import isaaclab.sim as sim_utils
from pxr import Gf, Sdf, UsdGeom, PhysxSchema, UsdPhysics, UsdShade

class WorkStationSetUp:
    def __init__(self,
                 desk_folder_path: str,
                 root_path="/World/Workstation",
                 desk_dynamic=False,
                 desk_mass=30.0,
                
                 desk_pose=((0.0, 0.0, 0.65), (0.0, 0.0, 0.0)),
                 surface_pose=((0.0, 0.0, 0.724), (0.0, 0.0, 0.0)),
                 legs_pose=((-0.2, 0.04, 0.65), (0.0, 0.0, 0.0)),
                 frame_pose= ((0.0, 0.0, 0.754), (0.0, 0.0, -90.0))):
        self._desk_folder_path = desk_folder_path
        self._root_path = Sdf.Path(root_path)
        self._stage = omni.usd.get_context().get_stage()

        self._desk_pose = desk_pose
        self._surface_pose = surface_pose
        self._legs_pose = legs_pose
        self._frame_pose = frame_pose

        self._desk_dynamic = desk_dynamic
        self._desk_mass = desk_mass
        self._desk_top_z = surface_pose[0][2]

        self._app = omni.kit.app.get_app()
        self._tick_sub = None

    def build(self):
        self._ensure_root()
        self._add_desk()
        self._add_scene()

        # subscribe to physics ticks (runs every frame)
        if self._tick_sub is None:
            self.tick_sub = self._app.get_update_event_stream() \
                .create_subscription_to_pop(self._on_update, name="workstation-tick")

    def _ensure_root(self):
        UsdGeom.Xform.Define(self._stage, self._root_path)

    def _add_pose(self, p1):
        # a function that add user input desk pose to default desk component pose
        (t1, r1) = p1
        (t2, r2) = self._desk_pose
        t = tuple(a+b for a, b in zip(t1, t2))
        r = tuple(a+b for a, b in zip(r1, r2))
        return (t, r)

    def _add_desk(self):
        desk_root = UsdGeom.Xform.Define(self._stage, self._root_path.AppendChild("Desk"))
        
        component_list = []
        for c in Path(self._desk_folder_path).glob("*.usda"):
            component_list.append(c)

        for usda_file in component_list:
            comp_root = UsdGeom.Xform.Define(self._stage, desk_root.GetPath().AppendChild(usda_file.stem))
            root_prim = comp_root.GetPrim()

            # Reference the asset under the wrapper
            asset_xf = UsdGeom.Xform.Define(self._stage, comp_root.GetPath().AppendChild("Asset"))
            asset_xf.GetPrim().GetReferences().AddReference(str(usda_file))

            # Pose the desk
            if ("desk_surface" in str(usda_file)):
                (tx, ty, tz), (r, p, y) = self._add_pose(self._surface_pose)
            elif("frame" in str(usda_file)):
                (tx, ty, tz), (r, p, y) = self._add_pose(self._frame_pose)
            elif("legs" in str(usda_file)):
                (tx, ty, tz), (r, p, y) = self._add_pose(self._legs_pose)

            R = Gf.Rotation(Gf.Vec3d(1,0,0), r) * Gf.Rotation(Gf.Vec3d(0,1,0), p) * Gf.Rotation(Gf.Vec3d(0,0,1), y)
            q = R.GetQuat()
            comp_root.AddTranslateOp().Set(Gf.Vec3f(tx, ty, tz))
            comp_root.AddOrientOp().Set(Gf.Quatf(q.GetReal(), *q.GetImaginary()))

            # Add Physics
            self._apply_collision_recursive(asset_xf.GetPrim())

            if self._desk_dynamic:
                UsdPhysics.RigidBodyAPI.Apply(root_prim)
                m = UsdPhysics.MassAPI.Apply(root_prim)
                m.CreateMassAttr(self._desk_mass)

            # bind materials

    def _apply_collision_recursive(self, prim):
        stack = [prim]
        while stack:
            p = stack.pop()
            t = p.GetTypeName()
            if t in ("Mesh", "Cube", "Cone", "Cylinder", "Sphere"):
                if not UsdPhysics.CollisionAPI(p):
                    UsdPhysics.CollisionAPI.Apply(p)
            for child in p.GetChildren():
                stack.append(child)


    def _add_scene(self):
        #Ground-plane
        cfg_ground = sim_utils.GroundPlaneCfg()
        cfg_ground.func("/World/defaultGroundPlane", cfg_ground)

        #spawn distant light
        cfg_light_distant = sim_utils.DistantLightCfg(
            intensity=3000.0,
            color=(0.75,0.75,0.75),
        )
        cfg_light_distant.func("/World/lightDistant", cfg_light_distant, translation=(1,0,10))

        # adding simple room environment scene
        room_path = "/World/room"
        room_xf = UsdGeom.Xform.Define(self._stage, room_path)
        room_prim = room_xf.GetPrim()

        simpleRoom_file_path = "/home/zdli/Arm-Simulation/arm/asset/simple_room_wNoTable.usda"
        room_prim.GetReferences().AddReference(simpleRoom_file_path)
        room_xf.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 0.78))


    def _on_update(self, _e):
        # Called every frame by Kit. If you need fixed-Δt logic, you can accumulate time here.
        # Example: lightweight task monitor / success checker.
        # dt = self._app.get_hydra_engine().get_dt()  # optional (engine-dependent)
        pass