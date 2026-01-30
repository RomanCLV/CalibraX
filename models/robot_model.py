from PyQt5.QtCore import QObject, pyqtSignal
from typing import List, Tuple
import math
from mgi import *

class RobotModel(QObject):
    """Modèle centralisé contenant tous les paramètres et l'état du robot"""
    
    # ============================================================================
    # SIGNAUX
    # ============================================================================
    
    # Configuration générale
    configuration_changed = pyqtSignal()
    robot_name_changed = pyqtSignal(str)
    
    # Paramètres DH
    dh_params_changed = pyqtSignal()
    
    # Joints et axes
    joints_changed = pyqtSignal()
    axis_reversed_changed = pyqtSignal()
    limits_changed = pyqtSignal()
    home_position_changed = pyqtSignal()
    
    # Corrections
    corrections_changed = pyqtSignal()
    
    # Résultats (TCP et cinématique)
    tcp_pose_changed = pyqtSignal()
    corrected_tcp_pose_changed = pyqtSignal()
    pose_deviation_changed = pyqtSignal()
    
    # Mesures
    measurements_changed = pyqtSignal()
    measurement_points_changed = pyqtSignal()
    
    def __init__(self, parent: QObject = None):
        super().__init__(parent)
        
        # ====================================================================
        # RÉGION: Configuration générale
        # ====================================================================
        self.robot_name = ""
        self.has_confifiguration = False
        self.current_config_file = None

        # ====================================================================
        # RÉGION: Paramètres des joints et axes
        # ====================================================================
        # Limites des axes (min, max) pour chaque joint
        self.axis_limits: List[Tuple[float, float]] = [(-180, 180) for _ in range(6)]
               
        # Position home du robot
        self.home_position: List[float] = [0, -90, 90, 0, 90, 0]
        
        # Valeurs actuelles des joints (en degrés)
        self.joint_values: List[float] = [0, 0, 0, 0, 0, 0]
        
        # Multiplicateurs d'axes (1 = normal, -1 = inversé)
        self.axis_reversed: List[int] = [1, 1, 1, 1, 1, 1]
        
        # ====================================================================
        # RÉGION: Paramètres Denavit-Hartenberg
        # ====================================================================
        # 7 lignes : 6 joints + outil (tool frame)
        # Chaque ligne contient [a, alpha, d, theta]
        self.dh_params: List[List[float]] = [[0, 0, 0, 0] for _ in range(7)]
        
        # ====================================================================
        # REGION: MGI
        # ====================================================================

        self.mgi_kuka_config_identifier = KukaConfigurationIdentifier()

        self.mgi_params = MgiParams(
            self.mgi_kuka_config_identifier, 
            RobotModel._mgi_build_geometric_params(self.dh_params), 
            RobotModel._mgi_build_invert_table(self.axis_reversed), 
            RobotModel._mgi_build_axis_limits(self.axis_limits), 
            MgiSingularitiesBehavior(MgiSingularityBehavior.CONTINUE),
            MgiConfigurationFilter.allow_all())
        
        self.MGI_solver = MGI(self.mgi_params, None)

        # ====================================================================
        # RÉGION: Corrections 6D
        # ====================================================================
        # 6 lignes pour 6 joints, 6 colonnes pour 6 DDL (X, Y, Z, Rx, Ry, Rz)
        self.corrections: List[List[float]] = [[0, 0, 0, 0, 0, 0] for _ in range(6)]
        
        # ====================================================================
        # RÉGION: Résultats cinématique
        # ====================================================================
        # Pose TCP non corrigée [X, Y, Z, Rx, Ry, Rz]
        self.tcp_pose: List[float] = [0, 0, 0, 0, 0, 0]
        
        # Pose TCP corrigée (appliquant les corrections)
        self.corrected_tcp_pose: List[float] = [0, 0, 0, 0, 0, 0]
        
        # Déviation entre TCP et TCP corrigé
        self.pose_deviation: List[float] = [0, 0, 0, 0, 0, 0]
        
        # ====================================================================
        # RÉGION: Mesures et points de mesure
        # ====================================================================
        # Liste des mesures enregistrées
        self.measurements: List[float] = []
        
        # Points de mesure (positions de référence)
        self.measurement_points: List[List[float]] = []

        self._compute_mgi_for_current_tcp()

        self._inhibit_compute_fk = False

    @staticmethod
    def _mgi_build_axis_limits(axis_limits: list[tuple[float, float]]):
        return MgiAxisLimits(False, axis_limits)

    @staticmethod
    def _mgi_build_invert_table(axis_reversed: list[int]):
        return [b == -1 for b in axis_reversed]

    @staticmethod
    def _mgi_build_geometric_params(dh_table: List[List[float]]) -> MgiGeometricParams:
        """
        Construit les paramètres géométriques du MGI à partir
        d'une table DH standard [a, alpha, d, theta].

        Hypothèse :
        - Robot 6 axes anthropomorphique
        - dh_table contient au moins 6 lignes - 7e ligne : pour le Tool
        """

        if len(dh_table) < 6:
            raise ValueError("La table DH doit contenir au moins 6 lignes")

        # Raccourcis de lecture
        # dh_table[i] = [a_i, alpha_i, d_i, theta_i]

        # Base  Axe 1
        r1 = dh_table[0][3]   # d1

        # Axe 2
        d2 = dh_table[1][1]   # d2

        # Axe 3
        d3 = dh_table[2][1]   # d3

        # Axe 4
        d4 = dh_table[3][1]   # d4
        r4 = dh_table[3][3]   # a4

        # Axe 6 / flange
        r6 = dh_table[5][3]   # d6

        return MgiGeometricParams(r1, d2, d3, d4, r4, r6)
    
    @staticmethod
    def _mgi_build_tool(dh_table: List[List[float]]) -> RobotTool:
        # TODO : Tool from dh_table
        return RobotTool()

    def compute_mgi(self, x: float, y: float, z: float, a: float, b: float, c: float):
        self.MGI_solver.set_q1ValueIfSingularityQ1(self.joint_values[0])
        self.MGI_solver.set_q4ValueIfSingularityQ5(self.joint_values[4])
        return self.MGI_solver.compute_mgi(x, y, z, a, b, c)

    def compute_mgi_target(self, target: list[float]):
        return self.compute_mgi(target[0], target[1], target[2], target[3], target[4], target[5])

    def _compute_mgi_for_current_tcp(self):
        self.current_mgi_result = self.compute_mgi_target(self.tcp_pose)

        # TODO : Selectionner la meilleure solution MGI et appliquer les joints
        
        self._update_current_axis_config()
        
        self.corrected_tcp_pose = list(self.tcp_pose) # TODO : Mettre à jour le TCP corrigé
        self._compute_deviation()
        self.corrected_tcp_pose_changed.emit()

    def get_best_mgi_solution(self, mgi_result: MgiResult):
        joints_rad = [math.radians(q) for q in self.joint_values]
        return mgi_result.get_best_solution_from_current(joints_rad, self.mgi_kuka_config_identifier, True)

    def get_config_identifier(self):
        return self.mgi_kuka_config_identifier
    
    def get_current_axis_config(self):
        return self.current_axis_config

    def _update_current_axis_config(self):
        self.current_axis_config = MgiConfigKey.identify_configuration(self.get_joints_not_inverted(), self.mgi_kuka_config_identifier)

    def compute_forward_kinematics(self):
        self._update_current_axis_config()

    def compute_forward_kinematics_corrected(self, compute_deviation: bool=True):
        if compute_deviation:
            self._compute_deviation()

    def compute_both_forward_kinematics(self):
        if self._inhibit_compute_fk:
            return
        self.compute_forward_kinematics()
        self.compute_forward_kinematics_corrected()
    
    # ============================================================================
    # RÉGION: Getters - Configuration générale
    # ============================================================================
    
    def get_robot_name(self) -> str:
        """Retourne le nom du robot"""
        return self.robot_name
    
    def get_has_confifiguration(self) -> bool:
        """Retourne True si le robot a une configuration chargée"""
        return self.has_confifiguration

    def get_current_config_file(self) -> str:
        """Retourne le chemin du fichier de configuration actuel"""
        return self.current_config_file
    
    # ============================================================================
    # RÉGION: Setters - Configuration générale
    # ============================================================================
    
    def set_robot_name(self, name: str):
        """Définit le nom du robot"""
        self.robot_name = name
        self.robot_name_changed.emit(name)
        
    # ============================================================================
    # RÉGION: Getters - Joints et axes
    # ============================================================================
    
    def get_joint(self, index: int) -> float:
        """Retourne la valeur d'un joint spécifique ou 0 si l'index est invalide"""
        return self.joint_values[index] if 0 <= index < 6 else 0
    
    def get_joints(self):
        """Retourne toutes les valeurs des joints"""
        return self.joint_values.copy()

    def get_joint_not_inverted(self, index: int) -> float:
        """Retourne la valeur d'un joint spécifique ou 0 si l'index est invalide"""
        return self.get_joint(index) * self.axis_reversed[index]

    def get_joints_not_inverted(self):
        """Retourne toutes les valeurs des joints"""
        return [self.joint_values[i] * self.axis_reversed[i] for i in range(6)]

    def get_axis_limit(self, index: int):
        """Retourne les limites d'un axe spécifique (min, max)"""
        return self.axis_limits[index] if 0 <= index < 6 else (-180, 180)
    
    def get_axis_limits(self):
        """Retourne les limites de tous les axes"""
        return self.axis_limits.copy()

    def get_axis_reversed_single(self, index: int) -> bool:
        """Retourne True si l'axe est inversé"""
        return self.axis_reversed[index] == -1 if 0 <= index < 6 else False

    def get_axis_reversed(self):
        """Retourne les multiplicateurs d'axes"""
        return self.axis_reversed.copy()
    
    def get_home_position(self):
        """Retourne la position home"""
        return self.home_position.copy()

    # ============================================================================
    # RÉGION: Setters - Joints et axes
    # ============================================================================
    
    def set_joint(self, index: int, value: float):
        """Modifie la valeur d'un joint spécifique"""
        if 0 <= index < 6:
            self.joint_values[index] = float(value)
            self.joints_changed.emit()
            self.compute_both_forward_kinematics()
    
    def set_joints(self, values: list[float]):
        """Définit toutes les valeurs des joints"""
        if len(values) >= 6:
            self.joint_values = list(values[:6])
            self.joints_changed.emit()
            self.compute_both_forward_kinematics()
        
    def set_axis_limit(self, index: int, min_val: float, max_val: float):
        """Définit les limites d'un axe spécifique"""
        if 0 <= index < 6:
            self.axis_limits[index] = (min_val, max_val)
            self.MGI_solver.set_axis_limits(RobotModel._mgi_build_axis_limits(self.axis_limits))
            # TODO : Les limites d'axes affectent elles la FK ?
            self.limits_changed.emit()
            self.compute_both_forward_kinematics()

    def set_axis_limits(self, limits: List[Tuple[float, float]]):
        """Définit les limites de tous les axes"""
        self.axis_limits = limits
        self.MGI_solver.set_axis_limits(RobotModel._mgi_build_axis_limits(self.axis_limits))
        # TODO : Les limites d'axes affectent elles la FK ?
        self.limits_changed.emit()
        self.compute_both_forward_kinematics()

    def set_axis_reversed_single(self, index: int, reversed_value: bool):
        """Inverse un axe spécifique"""
        if 0 <= index < 6:
            self.axis_reversed[index] = -1 if reversed_value else 1
            self.MGI_solver.set_invert_table(RobotModel._mgi_build_invert_table(self.axis_reversed))
            self.axis_reversed_changed.emit()
            self.compute_both_forward_kinematics()
        
    def set_axis_reversed(self, axis_reversed: list[int]):
        """Définit les multiplicateurs d'axes (1 ou -1)"""
        if len(axis_reversed) >= 6:
            self.axis_reversed = list(axis_reversed[:6])
            self.MGI_solver.set_invert_table(RobotModel._mgi_build_invert_table(self.axis_reversed))
            self.axis_reversed_changed.emit()
            self.compute_both_forward_kinematics()
    
    def set_home_position(self, home_pos: list[float]):
        """Définit la position home"""
        if len(home_pos) >= 6:
            self.home_position = list(home_pos[:6])
            self.home_position_changed.emit()
    
    # ============================================================================
    # RÉGION: Getters - Paramètres DH
    # ============================================================================
    
    def get_dh_params(self):
        """Retourne tous les paramètres DH"""
        return [row.copy() for row in self.dh_params]
    
    def get_dh_param(self, row: int, col: int):
        """Retourne un paramètre DH spécifique"""
        return self.dh_params[row][col] if 0 <= row < 7 and 0 <= col < 4 else 0
    
    def get_dh_row(self, row: int):
        """Retourne une ligne complète de paramètres DH"""
        return self.dh_params[row].copy() if 0 <= row < 7 else [0, 0, 0, 0]
    
    # ============================================================================
    # RÉGION: Setters - Paramètres DH
    # ============================================================================
    
    def set_dh_params(self, params: list[list[float]]):
        """Définit tous les paramètres DH"""
        self.dh_params = [list(row) for row in params]
        # Assurer 7 lignes
        while len(self.dh_params) < 7:
            self.dh_params.append([0, 0, 0, 0])
        self.dh_params = self.dh_params[:7]
        self.dh_params_changed.emit()
        self.MGI_solver.set_geometric_params(RobotModel._mgi_build_geometric_params(self.dh_params))
        self.compute_both_forward_kinematics()
    
    def set_dh_param(self, row: int, col: int, value: float):
        """Définit un paramètre DH spécifique"""
        if 0 <= row < 7 and 0 <= col < 4:
            try:
                self.dh_params[row][col] = float(value)
                self.dh_params_changed.emit()
                self.MGI_solver.set_geometric_params(RobotModel._mgi_build_geometric_params(self.dh_params))
                self.compute_both_forward_kinematics()

            except (ValueError, TypeError):
                print(f"Erreur: valeur DH invalide [{row},{col}] = {value}")
        else:
            print(f"Erreur: index DH invalide [{row},{col}]")
    
    def set_dh_row(self, row: int, values: list[float]):
        """Définit une ligne complète de paramètres DH"""
        if 0 <= row < 7 and len(values) >= 4:
            try:
                self.dh_params[row] = [float(v) for v in values[:4]]
                self.dh_params_changed.emit()
                self.MGI_solver.set_geometric_params(RobotModel._mgi_build_geometric_params(self.dh_params))
                self.compute_both_forward_kinematics()

            except (ValueError, TypeError):
                print(f"Erreur: valeurs DH invalides pour la ligne {row}")
    
    # ============================================================================
    # RÉGION: Getters - Corrections
    # ============================================================================
    
    def get_corrections(self):
        """Retourne toutes les corrections 6D"""
        return [row.copy() for row in self.corrections]
    
    def get_correction(self, row: int, col: int):
        """Retourne une correction 6D spécifique"""
        return self.corrections[row][col] if 0 <= row < 6 and 0 <= col < 6 else 0
    
    def get_correction_row(self, row: int):
        """Retourne une ligne complète de corrections"""
        return self.corrections[row].copy() if 0 <= row < 6 else [0, 0, 0, 0, 0, 0]
    
    def get_correction_joint(self, joint_index: int):
        """Retourne le vecteur de correction 6D pour un joint"""
        return self.corrections[joint_index].copy() if 0 <= joint_index < 6 else [0, 0, 0, 0, 0, 0]
    
    # ============================================================================
    # RÉGION: Getters - Résultats cinématique
    # ============================================================================
    
    def get_tcp_pose(self):
        """Retourne la pose TCP non corrigée"""
        return self.tcp_pose.copy()
    
    def get_corrected_tcp_pose(self):
        """Retourne la pose TCP corrigée"""
        return self.corrected_tcp_pose.copy()
    
    def get_pose_deviation(self):
        """Retourne la déviation entre TCP et TCP corrigé"""
        return self.pose_deviation.copy()
    
    def get_tcp_position(self):
        """Retourne la position (X, Y, Z) du TCP"""
        return self.tcp_pose[:3]
    
    def get_tcp_rotation(self):
        """Retourne la rotation (Rx, Ry, Rz) du TCP"""
        return self.tcp_pose[3:6]
    
    # ============================================================================
    # RÉGION: Setters - Résultats cinématique
    # ============================================================================
    
    def set_tcp_pose(self, pose: list[float]):
        """Définit la pose TCP non corrigée"""
        if len(pose) >= 6:
            self.tcp_pose = list(pose[:6])
            self.tcp_pose_changed.emit()
            self._compute_mgi_for_current_tcp()
    
    def _compute_deviation(self):
        """Calcule la déviation entre TCP et TCP corrigé"""
        self.pose_deviation = [
            self.corrected_tcp_pose[i] - self.tcp_pose[i] for i in range(6)
        ]
        self.pose_deviation_changed.emit()
    
    # ============================================================================
    # RÉGION: Getters - Mesures
    # ============================================================================
    
    def get_measurements(self):
        """Retourne la liste des mesures enregistrées"""
        return self.measurements.copy()
    
    def get_measurement(self, index: int):
        """Retourne une mesure spécifique"""
        return self.measurements[index] if 0 <= index < len(self.measurements) else None
    
    def get_measurement_count(self):
        """Retourne le nombre de mesures enregistrées"""
        return len(self.measurements)
    
    def get_measurement_points(self):
        """Retourne la liste des points de mesure"""
        return self.measurement_points.copy()
    
    def get_measurement_point(self, index):
        """Retourne un point de mesure spécifique"""
        return self.measurement_points[index] if 0 <= index < len(self.measurement_points) else  None
    
    # ============================================================================
    # RÉGION: Setters - Mesures
    # ============================================================================
    
    def add_measurement(self, measurement: float):
        """Ajoute une nouvelle mesure"""
        self.measurements.append(measurement)
        self.measurements_changed.emit()
        # TODO : Mettre à jour les corrections
        # TODO : Mettre à jour le TCP corrigé

    def add_measurement_point(self, point: list[float]):
        """Ajoute un nouveau point de mesure"""
        self.measurement_points.append(point)
        self.measurement_points_changed.emit()
        # TODO : Mettre à jour les corrections
        # TODO : Mettre à jour le TCP corrigé

    def set_measurements(self, measurements: list[float]):
        """Définit la liste des mesures"""
        self.measurements = list(measurements)
        self.measurements_changed.emit()
        # TODO : Mettre à jour les corrections
        # TODO : Mettre à jour le TCP corrigé
    
    def set_measurement_points(self, points: list[list[float]]):
        """Définit la liste des points de mesure"""
        self.measurement_points = list(points)
        self.measurement_points_changed.emit()
        # TODO : Mettre à jour les corrections
        # TODO : Mettre à jour le TCP corrigé
    
    def clear_measurements(self):
        """Efface toutes les mesures"""
        self.measurements.clear()
        self.measurements_changed.emit()
        # TODO : Mettre à jour les corrections
        # TODO : Mettre à jour le TCP corrigé
    
    def clear_measurement_points(self):
        """Efface tous les points de mesure"""
        self.measurement_points.clear()
        self.measurement_points_changed.emit()
        # TODO : Mettre à jour les corrections
        # TODO : Mettre à jour le TCP corrigé
    
    # ============================================================================
    # RÉGION: Sérialisation / Désérialisation
    # ============================================================================
    
    def to_dict(self):
        """Export vers dictionnaire (pour sauvegarde JSON)"""
        return {
            "name": [self.robot_name],
            "dh": [[str(val) for val in row] for row in self.dh_params[:6]],
            "corr": [[str(val) for val in row] for row in self.corrections],
            "q": self.joint_values,
            "axis_limits": self.axis_limits,
            "axis_reversed": self.axis_reversed,
            "home_position": self.home_position,
        }
    
    def load_from_dict(self, data: str, file_name: str=None):
        """Import depuis dictionnaire (pour chargement JSON)"""
        self._inhibit_compute_fk = True
        
        # Fichier de configuration
        if file_name:
            self.current_config_file = file_name

        # Nom du robot
        if "name" in data and len(data["name"]) > 0:
            self.robot_name = data["name"][0]
        
        # Paramètres DH
        if "dh" in data:
            dh_list = [[float(val) if val else 0 for val in row] for row in data["dh"]]
            while len(dh_list) < 7:
                dh_list.append([0, 0, 0, 0])
            self.set_dh_params(dh_list)
        
        # Corrections
        if "corr" in data:
            corr_list = [[float(val) if val else 0 for val in row] for row in data["corr"]]
            while len(corr_list) < 6:
                corr_list.append([0, 0, 0, 0, 0, 0])
            self.set_corrections(corr_list)
        
        # Valeurs des joints
        if "q" in data:
            self.set_joints(data["q"])
        
        # Multiplicateurs d'axes
        if "axis_reversed" in data:
            self.set_axis_reversed(list(data["axis_reversed"]))
        
        # Limites des axes
        if "axis_limits" in data:
            self.set_axis_limits(list(data["axis_limits"]))
        
        # Position home
        if "home_position" in data:
            self.set_home_position(data["home_position"])
            
        self.has_confifiguration = True

        # Émettre le signal de configuration changée
        self.configuration_changed.emit()

        self._inhibit_compute_fk = False
        self.compute_both_forward_kinematics()
