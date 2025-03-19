import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu
import math
import numpy as np
import time

class WamvPIDController(Node):
    def __init__(self):
        super().__init__('wamv_pid_controller')

        # 🚀 **PID Ultra-Agressif**
        self.kp = 8.0  # ⚡ Correction ultra-rapide
        self.ki = 0.1  # 🔧 Stabilisation minimale
        self.kd = 4.0  # 💥 Stoppe immédiatement les oscillations

        self.prev_error = 0.0
        self.integral = 0.0
        
        # 📍 Coordonnées de Spawn
        self.initial_lat = 42.35821841063174
        self.initial_lon = -71.04793301432125

        # 🎯 Nouvelle Cible
        self.target_lat = 42.3585  
        self.target_lon = -71.0485  

        self.returning = False
        self.yaw = 0.0  
        self.reached_time = None  # Ajout pour le temporisateur
        self.home_reached = False  # Flag pour savoir si on est revenu à la maison

        # Seuils d'erreur
        self.heading_tolerance = math.radians(25)  # 🔥 Augmenté pour éviter trop d'alignements inutiles

        # Vitesse max des moteurs
        self.max_thrust = 10.0  
        self.launch_boost = True  

        # Publishers
        self.left_thruster_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thruster_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        # Subscribers
        self.gps_sub = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)

        self.get_logger().info(f"🎯 Distance initiale vers la cible : {self.compute_distance(self.initial_lat, self.initial_lon, self.target_lat, self.target_lon):.2f} m")

    def imu_callback(self, msg):
        """ Met à jour le cap IMU sans l'afficher """
        self.yaw = self.quaternion_to_yaw(msg.orientation)

    def compute_distance(self, lat1, lon1, lat2, lon2):
        delta_lat = (lat2 - lat1) * 111000
        delta_lon = (lon2 - lon1) * 111000 * math.cos(math.radians(lat1))
        return math.sqrt(delta_lat**2 + delta_lon**2)

    def gps_callback(self, msg):
        current_lat = msg.latitude
        current_lon = msg.longitude

        distance = self.compute_distance(current_lat, current_lon, self.target_lat, self.target_lon)

        self.get_logger().info(f"📍 Position Actuelle: ({current_lat:.6f}, {current_lon:.6f}) | 🎯 Cible: ({self.target_lat:.6f}, {self.target_lon:.6f})")
        self.get_logger().info(f"📏 Distance à parcourir: {distance:.2f} m")

        if self.has_reached_target(current_lat, current_lon):
            if not self.returning:
                if self.reached_time is None:  # Si le robot atteint la cible pour la première fois
                    self.reached_time = time.time()  # Marquer l'heure d'arrivée
                    self.get_logger().info("✅ PHASE: ARRIVÉE | CIBLE ATTEINTE !")
                elif time.time() - self.reached_time >= 5:  # Si 5 secondes se sont écoulées
                    self.get_logger().info("⏱️ PHASE: GOHOME | Retour à la position initiale")
                    self.target_lat = self.initial_lat  # Réinitialiser la cible pour revenir à la position initiale
                    self.target_lon = self.initial_lon
                    self.reached_time = None  # Réinitialiser le temporisateur
                    self.returning = True
                return
            else:
                if not self.home_reached:  # Vérifie si on est revenu à la maison
                    self.get_logger().info("🏠 PHASE: ARRIVÉE À LA MAISON | Retour complet !")
                    self.home_reached = True
                    return

        distance, heading_error = self.compute_navigation(current_lat, current_lon)

        if distance < 5.0:
            return  

        if abs(heading_error) > self.heading_tolerance:
            self.get_logger().info(f"🔄 PHASE: ALIGNEMENT RAPIDE | Erreur de cap: {math.degrees(heading_error):.2f}°")
            turn_command = self.pid_controller(heading_error) * 1.5  
            self.send_thruster_commands(0, turn_command)
            return  

        if self.launch_boost:
            self.get_logger().info("🚀 PHASE: DÉPART ULTRA RAPIDE | Mode turbo activé !")
            turn_command = self.pid_controller(heading_error)
            self.send_thruster_commands(self.max_thrust, turn_command)
            self.launch_boost = False  
            return

        self.get_logger().info(f"🚀 PHASE: EN ROUTE (Ultra-Réactif) | Correction Express du Cap | Distance restante: {distance:.2f} m")
        turn_command = self.pid_controller(heading_error)
        self.send_thruster_commands(distance, turn_command)

    def has_reached_target(self, current_lat, current_lon):
        return self.compute_distance(current_lat, current_lon, self.target_lat, self.target_lon) < 5.0  

    def compute_navigation(self, current_lat, current_lon):
        delta_lat = (self.target_lat - current_lat) * 111000
        delta_lon = (self.target_lon - current_lon) * 111000 * math.cos(math.radians(current_lat))

        distance = math.sqrt(delta_lat**2 + delta_lon**2)
        target_angle = math.atan2(delta_lat, delta_lon)

        current_heading = self.yaw  
        heading_error = target_angle - current_heading
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi  

        return distance, heading_error

    def pid_controller(self, error):
        """ PID Extrême pour une correction immédiate """
        self.integral += error
        self.integral = max(-10, min(10, self.integral))
        derivative = error - self.prev_error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        return output

    def send_thruster_commands(self, distance, turn_command):
        max_thrust = self.max_thrust
        forward_thrust = min(distance / 10, max_thrust)

        if abs(turn_command) > 1.0:
            forward_thrust *= 0.7  

        min_forward_thrust = 2.0 if distance > 2.0 else 0.0  # 🔥 Minimum augmenté pour éviter la lenteur
        forward_thrust = max(forward_thrust, min_forward_thrust)

        left_thrust = forward_thrust - turn_command
        right_thrust = forward_thrust + turn_command

        if abs(turn_command) > math.radians(15) and distance < 2.0:  
            left_thrust = -turn_command
            right_thrust = turn_command

        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = max(-max_thrust, min(max_thrust, left_thrust))
        right_msg.data = max(-max_thrust, min(max_thrust, right_thrust))

        self.left_thruster_pub.publish(left_msg)
        self.right_thruster_pub.publish(right_msg)

        self.get_logger().info(f"🛠️ Thrusters -> Left: {left_msg.data}, Right: {right_msg.data}")

    def quaternion_to_yaw(self, quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def main(args=None):
    rclpy.init(args=args)
    controller = WamvPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
