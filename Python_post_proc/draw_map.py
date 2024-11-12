import numpy as np
import math
import cv2
import yaml
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point


class drawMap():
    """
    A class to handle the drawing of a map based on sensor measurements and robot coordinates.

    Attributes:
        image (ndarray): The image of the map loaded from a file.
        origin_x (float): The x-coordinate of the origin of the map.
        origin_y (float): The y-coordinate of the origin of the map.
        resolution (float): The resolution of the map.
        map_height (int): The height of the map in pixels.
        measurements (dict): Sensor measurements and robot poses loaded from a YAML file.

    Methods:
        transform(x_bot, y_bot, angle, ss, bs, fs): Transforms robot's base coordinates and orientation into two points.
        process_measurements(): Processes measurements to compute lines and their pixel coordinates.
        remove_duplicates(lines, tolerance): Removes duplicate lines based on a given tolerance.
        filter_noisy_measurements(lines, min_length): Filters out lines that are too short.
        draw_lines(filtered_points): Draws lines between given points.
        map(): Main mapping function that processes measurements and draws the map.
    """

    def __init__(self, map_img_file_path, map_yaml_file_path, measurements_yaml_file_path):

        self.image = cv2.imread(map_img_file_path, cv2.IMREAD_GRAYSCALE)

        with open(map_yaml_file_path, 'r') as file:
            map_metadata = yaml.safe_load(file)

        self.origin_x = map_metadata["origin"][0]
        self.origin_y = map_metadata["origin"][1]
        self.resolution = map_metadata["resolution"]
        self.map_height, _ = self.image.shape

        with open(measurements_yaml_file_path, 'r') as file1:
            self.measurements = yaml.safe_load(file1)

    def transform(self, x_bot, y_bot, angle, ss, bs, fs):
        """
        Transforms robot's base coordinates and orientation into two points based on sensor measurements.

        Returns:
        - tuple: Coordinates of two points (x1, y1, x2, y2).
        """
        x_robot = x_bot
        y_robot = y_bot
        theta_robot = np.radians(angle)

        x1 = round(x_robot + bs * math.cos(theta_robot) -
                   ss * math.sin(theta_robot), 2)
        y1 = round(y_robot + bs * math.sin(theta_robot) +
                   ss * math.cos(theta_robot), 2)

        x2 = round(x_robot + fs * math.cos(theta_robot) -
                   ss * math.sin(theta_robot), 2)
        y2 = round(y_robot + fs * math.sin(theta_robot) +
                   ss * math.cos(theta_robot), 2)

        return x1, y1, x2, y2


    def process_measurements(self):
        self.lines = []
        self.lengths = []

        for measure in self.measurements:
            if not measure:
                continue
            fs = measure["x1"]
            bs = measure["x2"]
            ss = measure["y1"]
            x = measure["robot_pose"]["x"]
            y = measure["robot_pose"]["y"]
            angle = measure["robot_pose"]["z"]

            x1, y1, x2, y2 = self.transform(x, y, angle, ss, bs, fs)
            length = fs - bs

            # Ignore points out of map bounds
            if not self.is_within_bounds(x1, y1) or not self.is_within_bounds(x2, y2):
                continue

            if all(not (math.isinf(c) or math.isnan(c)) for c in [x1, y1, x2, y2]):
                self.lines.append(((x1, y1), (x2, y2)))
                self.lengths.append(length)

        return self.lines, self.lengths

    def is_within_bounds(self, x, y):
        pixel_x = int((x - self.origin_x) / self.resolution)
        pixel_y = int((self.map_height - (y - self.origin_y) / self.resolution))
        return 0 <= pixel_x < self.image.shape[1] and 0 <= pixel_y < self.image.shape[0]

    def remove_duplicates(self, lines, tolerance=0.15):
        """Enhanced duplicate removal with better tolerance handling"""
        unique_lines = []
        for line in lines:
            line1 = LineString([line[0], line[1]])
            # Check both distance and angle between lines
            is_duplicate = False
            for existing_line in unique_lines:
                line2 = LineString([existing_line[0], existing_line[1]])
                if (line1.distance(line2) < tolerance and 
                    (abs(self._get_line_angle(line) - self._get_line_angle(existing_line)) < np.radians(5) or
                     abs(abs(self._get_line_angle(line) - self._get_line_angle(existing_line)) - np.pi) < np.radians(5))):
                    is_duplicate = True
                    break
            if not is_duplicate:
                unique_lines.append(line)
        return unique_lines

    def _get_line_angle(self, line):
        """Helper method to calculate line angle"""
        (x1, y1), (x2, y2) = line
        return np.arctan2(y2 - y1, x2 - x1)

    def filter_noisy_measurements(self, lines, min_length=0.5, max_length=15.0):
        """Enhanced filtering with maximum length and angle constraints"""
        filtered_lines = []
        for line in lines:
            line_obj = LineString([line[0], line[1]])
            length = line_obj.length
            if min_length <= length <= max_length:
                # Check if the line is roughly horizontal or vertical (within 3 degrees)
                angle = self._get_line_angle(line)
                if (abs(angle) < np.radians(3) or 
                    abs(abs(angle) - np.pi/2) < np.radians(3) or 
                    abs(abs(angle) - np.pi) < np.radians(3)):
                    filtered_lines.append(line)
        return filtered_lines

    def draw_lines(self, filtered_points):
        plt.figure(figsize=(12, 8))
        for ((x1, y1), (x2, y2)) in filtered_points:
            # Convert from meters to pixels
            pixel_x1 = x1 / self.resolution
            pixel_y1 = y1 / self.resolution
            pixel_x2 = x2 / self.resolution
            pixel_y2 = y2 / self.resolution
            
            plt.plot([x1, x2], [y1, y2], 'k-', linewidth=1.5)
            plt.scatter([x1, x2], [y1, y2], color='b', s=30)

        plt.gca().set_aspect('equal', adjustable='box')
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.title('Processed Map Lines')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        # Set reasonable axis limits based on the map dimensions
        plt.xlim([-8, 8])  # Adjust these values based on your map size
        plt.ylim([-6, 6])  # Adjust these values based on your map size
        plt.show()

    def connect_nearby_lines(self, lines, max_gap=0.3):
        """Connect lines that are close to each other and have similar angles"""
        connected_lines = []
        used = set()
        
        for i, line1 in enumerate(lines):
            if i in used:
                continue
                
            current_line = line1
            used.add(i)
            
            while True:
                found_connection = False
                line1_obj = LineString([current_line[0], current_line[1]])
                
                for j, line2 in enumerate(lines):
                    if j in used:
                        continue
                        
                    line2_obj = LineString([line2[0], line2[1]])
                    
                    # Check if lines have similar angles and are close to each other
                    if (abs(self._get_line_angle(current_line) - self._get_line_angle(line2)) < np.radians(5) and
                        line1_obj.distance(line2_obj) < max_gap):
                        
                        # Connect the lines
                        points = list(line1_obj.coords) + list(line2_obj.coords)
                        connected_line = ((min(p[0] for p in points), min(p[1] for p in points)),
                                       (max(p[0] for p in points), max(p[1] for p in points)))
                        
                        current_line = connected_line
                        used.add(j)
                        found_connection = True
                        break
                
                if not found_connection:
                    break
                    
            connected_lines.append(current_line)
        
        return connected_lines

    def map(self):
        """
        Main mapping function that processes measurements and draws the map.
        """
        lines, _ = self.process_measurements()
        unique_lines = self.remove_duplicates(lines)
        filtered_lines = self.filter_noisy_measurements(unique_lines)
        connected_lines = self.connect_nearby_lines(filtered_lines)
        self.draw_lines(connected_lines)
        
        return connected_lines


def main():

    mapper = drawMap(map_img_file_path="SLAM/slam_map.pgm", 
                     map_yaml_file_path="SLAM/slam_map.yaml", 
                     measurements_yaml_file_path="measurements.yaml")
    
    lines = mapper.map()


if __name__ == "__main__":
    main()
