import rclpy
from xml.etree import ElementTree

from rclpy.node import Node
from PIL import Image
from typing import Any

class osm_cartographer(Node):
    __rclpy_node_name__: str = "rclpy_osm_cartographer"
    __rclpy_flags__: str = "RCLPY"

    def __init__(self) -> None:
        super().__init__(self.__rclpy_node_name__)
        self.get_logger().info(
            "===== {} [{}] created =====".format(
                self.__rclpy_flags__, self.__rclpy_node_name__
            )
        )
        
        self.__osm_width__: int = 0
        self.__osm_height__: int = 0

        self.__draw_pgm__()
        
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().warn(
                "===== {} [{}] terminated with ctrl-c =====".format(
                    self.__rclpy_flags__, self.__rclpy_node_name__
                )
            )

        self.destroy_node()
    
    def __get_osm_width__(self) -> int:
        return self.__osm_width__
    
    def __set_osm_width__(self, osm_width: int) -> None:
        self.__osm_width__ = osm_width
    
    def __get_osm_height__(self) -> int:
        return self.__osm_height__
    
    def __set_osm_height__(self, osm_height: int) -> None:
        self.__osm_height__ = osm_height
    
    def __get_required_attribute__(self, el, key) -> Any:
        val: Any = el.get(key)
        if val is None:
            raise ValueError('required attribute missing: ' + key)
        return val

    def __save_pgm_image__(self, image: Image, file_name: str) -> None:
        with open(file_name, "wb") as f:
            f.write(b"P5\n")
            f.write(f"{self.__get_osm_width__()} {self.__get_osm_height__()}\n".encode())
            f.write(b"255\n")
            image.save(f, format="PPM")
            
    def __draw_pgm__(self) -> None:
        osm_file_path = "/home/wavem/ros2_ws/src/rclpy_osm_cartographer/src/rclpy_osm_cartographer/maps/seong_nam.osm"
        xm = None
        
        try:
            with open(osm_file_path, 'r') as f:
                xm = ElementTree.parse(f)
        except IOError:
            raise ValueError("{} unable to read [{}]".format(self.__rclpy_flags__, osm_file_path))
        except ElementTree.ParseError:
            raise ValueError("{} XML parse failed for [{}]".format(self.__rclpy_flags__, osm_file_path))
        
        osm = xm.getroot()
        
        for el in osm.iterfind('bounds'):
            min_lat: float = float(self.__get_required_attribute__(el, 'minlat'))
            min_lon: float = float(self.__get_required_attribute__(el, 'minlon'))
            max_lat: float = float(self.__get_required_attribute__(el, 'maxlat'))
            max_lon: float = float(self.__get_required_attribute__(el, 'maxlon'))
            
            width: int = int((max_lon - min_lon) * 100000)
            self.__set_osm_width__(width)
            
            height: int = int((max_lat - min_lat) * 100000)
            self.__set_osm_height__(height)
            
            self.get_logger().info("{} osm width : [{}], height : [{}]".format(self.__rclpy_flags__, str(width), str(height)))

        pgm_image = Image.new("L", (self.__get_osm_width__(), self.__get_osm_height__()), 255)
        
        for node in osm.iter("node"):
            lon: float = float(node.attrib["lon"])
            lat: float = float(node.attrib["lat"])
            
            self.get_logger().info("{} osm lon : [{}], lat : [{}]".format(self.__rclpy_flags__, str(lon), str(lat)))
            
            x: int = int((lon - min_lon) * 100000)
            y: int = int((lat - min_lat) * 100000)

            self.get_logger().info("{} osm x : [{}], y : [{}]".format(self.__rclpy_flags__, str(x), str(y)))
            
            is_way: bool = False
            for tag in node.iter("tag"):
                if tag.attrib["k"] == "way":
                    is_way = True
                    break

            color: int = 255 if is_way else 0
            
            if 0 <= x < self.__get_osm_width__() and 0 <= y < self.__get_osm_height__():
                pgm_image.putpixel((x, y), color)
        
        pgm_file_path = "/home/wavem/ros2_ws/src/rclpy_osm_cartographer/result/test.pgm"
        self.__save_pgm_image__(pgm_image.transpose(Image.FLIP_LEFT_RIGHT), pgm_file_path)
        self.get_logger().info("{} PGM image file created: {}".format(self.__rclpy_flags__, pgm_file_path))