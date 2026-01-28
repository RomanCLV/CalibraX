from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout
from RobotTab.widgets.correction_table_widget import CorrectionTableWidget
from RobotTab.widgets.joint_control_widget import JointControlWidget
from RobotTab.widgets.joints_result_table_widget import JointsResultTableWidget
from RobotTab.robotmodel import RobotModel

class JointControlWindow(QWidget):
    """Widget de contrôle articulaire du robot"""
    
    def __init__(self, robot_model: RobotModel, leftStrect=1, rightStrect=2, parent: QWidget = None):
        super().__init__(parent)
        # Initialisation des composants du widget de contrôle articulaire

        self.left_stretch = leftStrect
        self.right_stretch = rightStrect

        self.robot_model = robot_model
        self.joint_widget = JointControlWidget(self.robot_model)
        self.joints_result_widget = JointsResultTableWidget()
        self.correction_widget = CorrectionTableWidget()
        self._setup_ui()

    def _setup_ui(self):
        # Configuration de l'interface utilisateur du widget de contrôle articulaire
        self._layout = QHBoxLayout()

        left_layout = QVBoxLayout()
        left_layout.addWidget(self.joint_widget)
        left_layout.addWidget(self.joints_result_widget)
        left_layout.addWidget(self.correction_widget)
        #left_layout.addStretch()

        self._layout.addLayout(left_layout, self.left_stretch)

        self.setLayout(self._layout)

    def add_viewer(self, viewer_widget):
        """Ajoute le widget de visualisation 3D au layout"""
        self._layout.addWidget(viewer_widget, self.right_stretch)
    
    def pick_viewer(self):
        """Retourne le widget de visualisation 3D"""
        return self._layout.itemAt(1).widget()

    def remove_viewer(self):
        """Retire le widget de visualisation 3D du layout"""
        viewer_widget = self.pick_viewer()
        if viewer_widget:
            self._layout.removeWidget(viewer_widget)
            viewer_widget.setParent(None)
