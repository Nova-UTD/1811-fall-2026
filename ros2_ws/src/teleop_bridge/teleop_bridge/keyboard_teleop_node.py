"""
GUI-based keyboard teleop using pygame. Publishes vehicle_msgs/VehicleCommand
on /vehicle_command.

Unlike a terminal, pygame gives real KEYDOWN and KEYUP events, so "released"
is known exactly -- not guessed via a timeout. Held-down keys are tracked in
a set; the command is recomputed fresh from that set every tick, so nothing
can queue up or fall behind.

Controls:
  Arrow keys  - drive (throttle / steer)
  Shift       - brake (held = full brake, overrides throttle)
  +/-         - adjust speed scale
  q or Esc    - quit
Click into the teleop window first -- it only sees keys while it has focus.
"""
import pygame

import rclpy
from rclpy.node import Node
from vehicle_msgs.msg import VehicleCommand

WINDOW_W, WINDOW_H = 480, 320
PUBLISH_HZ = 10.0
SPEED_STEP = 0.1
MIN_SPEED_SCALE = 0.1
MAX_SPEED_SCALE = 1.0


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        self.pub = self.create_publisher(VehicleCommand, '/vehicle_command', 10)

        pygame.init()
        pygame.display.set_caption('Vehicle Teleop -- click here, use arrow keys')
        self.screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
        self.font = pygame.font.SysFont('monospace', 18)

        self.held_keys = set()
        self.speed_scale = 0.5
        self.shutting_down = False

        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.on_timer)
        self.get_logger().info('Teleop window opened. Click it, then use arrow keys.')

    def on_timer(self):
        if self.shutting_down:
            return

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.request_shutdown()
                return
            elif event.type == pygame.KEYDOWN:
                self.held_keys.add(event.key)
                if event.key in (pygame.K_EQUALS, pygame.K_PLUS):
                    self.speed_scale = min(MAX_SPEED_SCALE, self.speed_scale + SPEED_STEP)
                elif event.key == pygame.K_MINUS:
                    self.speed_scale = max(MIN_SPEED_SCALE, self.speed_scale - SPEED_STEP)
                elif event.key in (pygame.K_q, pygame.K_ESCAPE):
                    self.request_shutdown()
                    return
            elif event.type == pygame.KEYUP:
                self.held_keys.discard(event.key)

        braking = pygame.K_LSHIFT in self.held_keys or pygame.K_RSHIFT in self.held_keys
        brake = 1.0 if braking else 0.0

        throttle = 0.0
        steer = 0.0
        if not braking:
            if pygame.K_UP in self.held_keys:
                throttle += self.speed_scale
            if pygame.K_DOWN in self.held_keys:
                throttle -= self.speed_scale
        if pygame.K_LEFT in self.held_keys:
            steer -= 1.0
        if pygame.K_RIGHT in self.held_keys:
            steer += 1.0

        cmd = VehicleCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.throttle = max(-1.0, min(1.0, throttle))
        cmd.steer = max(-1.0, min(1.0, steer))
        cmd.brake = brake
        self.pub.publish(cmd)

        self.draw(cmd)

    def draw(self, cmd):
        self.screen.fill((20, 20, 20))
        lines = [
            "VEHICLE TELEOP",
            "",
            "Arrows: drive   Shift: brake   +/-: speed   q/Esc: quit",
            "",
            f"Speed scale: {self.speed_scale:.1f}",
            f"Throttle:    {cmd.throttle:+.2f}",
            f"Steer:       {cmd.steer:+.2f}",
            f"Brake:       {cmd.brake:.2f}",
        ]
        y = 20
        for line in lines:
            surf = self.font.render(line, True, (0, 255, 0))
            self.screen.blit(surf, (20, y))
            y += 28
        pygame.display.flip()

    def request_shutdown(self):
        # Publish one final zero command so the vehicle doesn't keep its
        # last command after the window closes.
        zero = VehicleCommand()
        zero.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(zero)
        self.shutting_down = True

    def destroy_node(self):
        pygame.quit()
        super().destroy_node()


def main():
    rclpy.init()
    node = KeyboardTeleopNode()
    try:
        while rclpy.ok() and not node.shutting_down:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
