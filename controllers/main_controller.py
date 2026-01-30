from models.robot_model import RobotModel
from controllers.robot_controller import RobotController
from controllers.joint_control_controller import JointControlController
from controllers.cartesian_control_controller import CartesianControlController
from views.main_window2 import MainWindow


class MainController:
    def __init__(self, robot_model: RobotModel, main_window: MainWindow):
        self.robot_model = robot_model
        self.main_window = main_window

        # controllers
        self.robot_controller = RobotController(robot_model, main_window.get_robot_view())
        self.joint_control_controller = JointControlController(robot_model, main_window.get_joint_control_view())
        self.cartesian_control_controller = CartesianControlController(robot_model, main_window.get_cartesian_control_view())

        self._setup_connections()

    def _setup_connections(self) -> None:
        """Configure les connexions de signaux entre la vue et le modèle du robot"""
        pass
    