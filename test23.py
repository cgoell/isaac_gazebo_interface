import carb.input
from pxr import Usd, UsdGeom
import omni.kit.commands
import omni.ext
import omni.appwindow
import omni.ui as ui
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription

import asyncio
import weakref
from omni.isaac.motion_planning import _motion_planning
from omni.isaac.dynamic_control import _dynamic_control
import omni.physx as _physx

import omni.isaac.demos
import omni.isaac.demos

EXTENSION_NAME = "Leonardo Preview"


class Extension(omni.ext.IExt):
    def on_startup(self):
        """Initialize extension and UI elements
        """
        self._timeline = omni.timeline.get_timeline_interface()
        self._viewport = omni.kit.viewport_legacy.get_default_viewport_window()
        self._usd_context = omni.usd.get_context()
        self._stage = self._usd_context.get_stage()
        self._window = None
        self._create_franka_btn = None
        self._perform_task_btn = None
        self._stop_task_btn = None
        self._toggle_obstacle_btn = None

        self._mp = _motion_planning.acquire_motion_planning_interface()
        self._dc = _dynamic_control.acquire_dynamic_control_interface()

        self._physxIFace = _physx.acquire_physx_interface()

        self._settings = carb.settings.get_settings()

        self._appwindow = omni.appwindow.get_default_app_window()
        self._sub_stage_event = self._usd_context.get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event
        )
        self._scenario = Scenario(self._dc, self._mp)
        self._editor_event_subscription = None

        self._menu_items = [
            MenuItemDescription(
                name="Demos",
                sub_menu=[
                    MenuItemDescription(
                        name="Leonardo Demo", onclick_fn=lambda a=weakref.proxy(self): a._menu_callback()
                    )
                ],
            )
        ]

        add_menu_items(self._menu_items, "Isaac Examples")

        self._first_step = True

    def _menu_callback(self):
        self._build_ui()

    def _build_ui(self):
        if not self._window:
            self._window = ui.Window(
                title=EXTENSION_NAME, width=300, height=200, dockPreference=ui.DockPreference.LEFT_BOTTOM
            )
            with self._window.frame:
                with ui.VStack():
                    self._create_franka_btn = ui.Button("Create Scenario", clicked_fn=self._on_environment_setup)

                    self._perform_task_btn = ui.Button("Perform Task", clicked_fn=self._on_perform_task)
                    self._perform_task_btn.enabled = False

                    self._stop_task_btn = ui.Button("Stop/Reset Task", clicked_fn=self._on_stop_tasks)
                    self._stop_task_btn.enabled = False

                    self._toggle_obstacle_btn = ui.Button("Toggle Obstacle", clicked_fn=self._on_toggle_obstacle)
                    self._toggle_obstacle_btn.enabled = False
            self._editor_event_subscription = (
                omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update_ui)
            )
        self._window.visible = True

    def _on_environment_setup(self):
        # wait for new stage before creating franka
        task = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        asyncio.ensure_future(self._on_create_franka(task))

    async def _on_create_franka(self, task):
        """Load any assets required by the scenario and create objects
        """
        done, pending = await asyncio.wait({task})
        if task not in done:
            await omni.kit.app.get_app().next_update_async()
            return

        self._stage = self._usd_context.get_stage()
        self._scenario = GhostScenario(self._dc, self._mp)

        self._first_step = True
        self._create_franka_btn.enabled = False

        self._timeline.stop()
        self._physxIFace.release_physics_objects()

        self._settings.set("/rtx/reflections/halfRes", True)
        self._settings.set("/rtx/shadows/denoiser/quarterRes", True)
        self._settings.set("/rtx/translucency/reflectionCutoff", 0.1)

        self._scenario.create_franka()

        self._physxIFace.release_physics_objects()
        self._physxIFace.force_load_physics_from_usd()

        self._physxIFace.release_physics_objects()
        self._physxIFace.force_load_physics_from_usd()

        self._physx_subs = _physx.get_physx_interface().subscribe_physics_step_events(self._on_simulation_step)
        self._stop_task_btn.enabled = True
        self._toggle_obstacle_btn.enabled = True

        self._viewport.set_camera_position("/OmniverseKit_Persp", 1.42, -1.27, 0.56, True)
        self._viewport.set_camera_target("/OmniverseKit_Persp", -1.80, 2.34, -0.27, True)

        light_prim = self._stage.GetPrimAtPath("/World/defaultLight")
        if light_prim:
            light_prim.SetActive(False)

    def _on_stop_tasks(self, *args):
        """Stop all tasks being performed by the scenario
        """
        self._scenario.stop_tasks()

    def _on_simulation_step(self, step):
        """This function is called every timestep in the editor

        Arguments:
            step (float): elapsed time between steps
        """
        if self._first_step:
            self._scenario.register_assets()
            self._first_step = False
        self._scenario.step(step)

    def _on_stage_event(self, event):
        """This function is called when stage events occur.
        Enables UI elements when stage is opened.
        Prevents tasks from being started until all assets are loaded

        Arguments:
            event (int): event type
        """
        if self._window:
            self.stage = self._usd_context.get_stage()
            if event.type == int(omni.usd.StageEventType.OPENED):
                self._create_franka_btn.enabled = True
                self._perform_task_btn.enabled = False
                self._stop_task_btn.enabled = False
                self._toggle_obstacle_btn.enabled = False
                self._timeline.stop()
                self._on_stop_tasks()
                self._scenario = Scenario(self._dc, self._mp)

    def _on_toggle_obstacle(self, *args):
        """Toggle obstacle visibility
        """
        for obstacle in self._scenario._obstacles:
            imageable = UsdGeom.Imageable(self._stage.GetPrimAtPath(obstacle.asset_path))
            visibility = imageable.ComputeVisibility(Usd.TimeCode.Default())
            if visibility == UsdGeom.Tokens.invisible:
                imageable.MakeVisible()
                obstacle.unsuppress()
            else:
                imageable.MakeInvisible()
                obstacle.suppress()

    def _on_perform_task(self, *args):
        """Perform all tasks in the scenario
        """
        self._scenario.perform_tasks()

    def _on_update_ui(self, step):
        """Callback that updates UI elements every frame
        """
        if self._scenario.is_created():
            self._create_franka_btn.enabled = False
            self._perform_task_btn.enabled = False
            self._stop_task_btn.enabled = False
            if self._timeline.is_playing():
                self._perform_task_btn.enabled = True
                self._perform_task_btn.text = "Perform Task"
                if self._scenario._running is True:
                    self._perform_task_btn.enabled = False
                    self._stop_task_btn.enabled = True
                else:
                    self._perform_task_btn.enabled = True
                    self._stop_task_btn.enabled = False
            else:
                self._perform_task_btn.enabled = False
                self._perform_task_btn.text = "Press Play To Enable"
                self._scenario._running = False
        else:
            self._create_franka_btn.enabled = True
            self._perform_task_btn.enabled = False
            self._perform_task_btn.text = "Press Create To Enable"
            self._stop_task_btn.enabled = False
            self._toggle_obstacle_btn.enabled = False

    def on_shutdown(self):
        """Cleanup objects on extension shutdown
        """
        self._timeline.stop()
        self._on_stop_tasks()
        self._scenario = None
        self._editor_event_subscription = None
        self._physx_subs = None
        remove_menu_items(self._menu_items, "Isaac Examples")
        self._window = None
        self._menus = None
