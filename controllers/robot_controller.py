from models.robot_model import RobotModel
from views.robot_view import RobotView


class RobotController:
    def __init__(self, robot_model: RobotModel, robot_view: RobotView):
        self.robot_model = robot_model
        self.robot_view = robot_view
        self._setup_connections()
        
    def _setup_connections(self) -> None:
        """Configure les connexions de signaux entre la vue et le modèle du robot"""
        pass
