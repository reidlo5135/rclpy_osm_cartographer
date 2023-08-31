import rclpy
from rclpy.exceptions import ROSInterruptException
from .cartographer.osm_cartographer import osm_cartographer


def main(args=None) -> None:
    rclpy.init(args=args)

    try:
        osm_cartographer()
    except ROSInterruptException:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
