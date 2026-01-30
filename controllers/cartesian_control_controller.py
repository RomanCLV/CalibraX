from models.robot_model import RobotModel
from views.cartesian_control_view import CartesianControlView


class CartesianControlController:
    def __init__(self, robot_model: RobotModel, cartesian_control_view: CartesianControlView):
        self.robot_model = robot_model
        self.cartesian_control_view = cartesian_control_view
        self._setup_connections()
    
    def _setup_connections(self) -> None:
        """Configure les connexions de signaux entre la vue et le modèle du robot"""
        pass