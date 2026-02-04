import numpy as np
from PyQt5.QtWidgets import QTableWidget

# ============================================================================
# RÉGION: Parsing et utilitaires
# ============================================================================

def parse_value(expr: str):
    """Parse une expression mathématique contenant potentiellement 'pi'"""
    
    try:
        expr = expr.replace("pi", "np.pi")
        return eval(expr, {"np": np})
    except Exception:
        raise ValueError(f"Expression invalide: {expr}")

def get_cell_value(table: QTableWidget, row: int, col: int, default=0):
    """Récupère la valeur d'une cellule de table Qt"""
    item = table.item(row, col)
    if item and item.text().strip() != "":
        return parse_value(item.text())
    return default

# ============================================================================
# RÉGION: Transformations Denavit-Hartenberg
# ============================================================================

def dh_modified(alpha: float, d: float, theta: float, r: float):
    """Calcule la matrice de transformation DH modifiée (4x4)
    
    Args:
        alpha: Angle de rotation autour de X (radians)
        d: Distance selon Z
        theta: Angle de rotation autour de Z (radians)
        r: Distance selon X
    
    Returns:
        Matrice homogène 4x4
    """
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct, -st, 0, d],
        [st*ca, ct*ca, -sa, -r*sa],
        [st*sa, ct*sa, ca, r*ca],
        [0, 0, 0, 1]
    ])

# ============================================================================
# RÉGION: Corrections 6D
# ============================================================================

def correction_6d(T, tx: float, ty: float, tz: float, rx: float, ry: float, rz: float):
    """Applique une correction 6D (translation + rotation ZYX) à une matrice homogène
    
    Args:
        T: Matrice homogène 4x4
        tx, ty, tz: Translation en mm
        rx, ry, rz: Rotation en degrés (ZYX Euler angles)
    
    Returns:
        Matrice homogène corrigée
    """
    rx, ry, rz = np.radians([rx, ry, rz])
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])
    R = Rz @ Ry @ Rx  # Rotation Fixed angles ZYX
    corr = np.eye(4)
    corr[:3, :3] = R
    corr[:3, 3] = [tx, ty, tz]
    return T @ corr

# ============================================================================
# RÉGION: Conversions angles d'Euler
# ============================================================================

def matrix_to_euler_zyx(T):
    """Extrait les angles d'Euler ZYX (en degrés) d'une matrice homogène 4x4
    
    Args:
        T: Matrice homogène 4x4
    
    Returns:
        Array [Rz, Ry, Rx] en degrés
    """
    rx = np.degrees(np.arctan2(T[2, 1], T[2, 2]))
    ry = np.degrees(np.arctan2(-T[2, 0], np.sqrt(T[2, 1]**2 + T[2, 2]**2)))
    rz = np.degrees(np.arctan2(T[1, 0], T[0, 0]))
    return np.array([rz, ry, rx])

def euler_to_rotation_matrix(A: float, B: float, C: float, degrees=True):
    """Convertit des angles d'Euler ZYX en matrice de rotation 3x3
    
    Args:
        A, B, C: Angles de rotation (degrés ou radians)
        degrees: Si True, les angles sont en degrés
    
    Returns:
        Matrice de rotation 3x3
    """
    if degrees:
        A, B, C = np.radians([A, B, C])
    
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(C), -np.sin(C)],
                   [0, np.sin(C), np.cos(C)]])
    Ry = np.array([[np.cos(B), 0, np.sin(B)],
                   [0, 1, 0],
                   [-np.sin(B), 0, np.cos(B)]])
    Rz = np.array([[np.cos(A), -np.sin(A), 0],
                   [np.sin(A), np.cos(A), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx

def rotation_matrix_to_euler_zyx(R):
    """Extrait les angles d'Euler ZYX (en degrés) d'une matrice de rotation 3x3
    
    Args:
        R: Matrice de rotation 3x3
    
    Returns:
        Array [A, B, C] en degrés
    """
    B = np.arcsin(-R[2, 0])
    A = np.arctan2(R[2, 1], R[2, 2])
    C = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees([A, B, C])
