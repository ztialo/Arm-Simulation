import math
import omni
import carb
import argparse
import random
from pathlib import Path

import numpy as np
import omni.kit.app, omni.usd
import isaaclab.sim as sim_utils
from pxr import Gf, Sdf, UsdGeom, PhysxSchema, UsdPhysics
# import PhysxSchemaPhysxConvexDecompositionCollisionAPI

class TaskSetUp:
    def __init__(self,
                 task_folder_path: str,
                 root_path="/World/Task",
                 desk_pose=((0.0, 0.0, 0.65), (0.0, 0.0, 0.0)),
                 desktop_height = 0.75):
        self._task_folder_path = task_folder_path
        self._root_path = Sdf.Path(root_path)
        self._stage = omni.usd.get_context().get_stage()

        self._desk_pose = desk_pose
        self._desktop_height = desktop_height

        self._app = omni.kit.app.get_app()
        self._tick_sub = None

    def build(self):
        self._ensure_root()
        self._add_props()

        # subscribe to physics ticks (runs every frame)
        if self._tick_sub is None:
            self.tick_sub = self._app.get_update_event_stream() \
                .create_subscription_to_pop(self._on_update, name="workstation-tick")

    def _ensure_root(self):
        UsdGeom.Xform.Define(self._stage, self._root_path)

    def _add_props(self):
        # iterate through the props list and add the props into the scene
        props_folder = Path(self._task_folder_path) / "props"

        files_list = []
        for usd_file in props_folder.glob("*.usd*"):
            # save the usd file on to a list
            files_list.append(usd_file)

        # Define an Xform prim at /World/Task/Props
        props_root = UsdGeom.Xform.Define(self._stage, self._root_path.AppendChild("Props"))

        for props_usd in files_list:
            # put props in xform wrapper to allow transform op
            item_root = UsdGeom.Xform.Define(self._stage, props_root.GetPath().AppendChild(props_usd.stem))
            root_prim = item_root.GetPrim()

            # Reference the asset under the wrapper
            asset_xf = UsdGeom.Xform.Define(self._stage, item_root.GetPath().AppendChild("Asset"))
            asset_xf.GetPrim().GetReferences().AddReference(str(props_usd))

            #randomize the prop's orientation and location on desk
            roll = random.uniform(-180.0, 180.0)
            pitch = random.uniform(-90.0, 90.0)
            yaw = random.uniform(-180.0, 180.0)

            desk_center_x = self._desk_pose[0][0]
            desk_center_y = self._desk_pose[0][1]
            tx = random.uniform(desk_center_x-0.3, desk_center_x+0.3)
            ty = random.uniform(desk_center_y-0.08, desk_center_y+0.3)

            R = Gf.Rotation(Gf.Vec3d(1,0,0), roll) * Gf.Rotation(Gf.Vec3d(0,1,0), pitch) * Gf.Rotation(Gf.Vec3d(0,0,1), yaw)
            q = R.GetQuat()
            item_root.AddTranslateOp().Set(Gf.Vec3f(tx, ty, self._desktop_height+0.2))
            item_root.AddOrientOp().Set(Gf.Quatf(q.GetReal(), *q.GetImaginary()))

            # apply rigid body and collision
            PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(asset_xf.GetPrim())
            UsdPhysics.RigidBodyAPI.Apply(root_prim)
            m = UsdPhysics.MassAPI.Apply(root_prim)
            m.CreateMassAttr(0.5) # Need to find a way to assign specific masss to relative object
            self._apply_collision_recursive(asset_xf.GetPrim())

    def _apply_collision_recursive(self, prim):
        stack = [prim]
        while stack:
            p = stack.pop()
            t = p.GetTypeName()
            if t in ("Mesh", "Cube", "Cone", "Cylinder", "Sphere"):
                collision = UsdPhysics.CollisionAPI(p)
                if not collision or not collision.IsApplied():
                    UsdPhysics.CollisionAPI.Apply(p)

                # meshCollision = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(p)
                # meshCollision.CreateSdfResolutionAttr().Set(256)

                # # set PhysX collision approximation as convex instead of triangle mesh 
                # PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(p)

            # apply collision to prim's child if there are any
            for child in p.GetChildren():
                stack.append(child)

    def _on_update(self, _e):
        
        pass