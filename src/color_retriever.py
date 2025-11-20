import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# Bilateral filter parameters
sigmaColour = 9
sigmaSpace = 75

# Color tolerance for HSV masking
TOLERANCE_H = 10
TOLERANCE_S = 20
TOLERANCE_V = 20

class ColorSelector(Node):
    def __init__(self):
        super().__init__('color_selector')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create image subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Initialize color storage
        self.colors = {
            'G': [],  # Green  
            'B': [],  # Blue
            'Y': [],  # Yellow
            'P': []   # Pink
        }
        # Current active color for selection
        self.current_color = 'B'
        self.current_image = None
        # Create window and set mouse callback
        cv2.namedWindow('Color Selector', cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback('Color Selector', self.mouse_callback)
        cv2.namedWindow('Color results', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('Text Display', cv2.WINDOW_AUTOSIZE)
        
        self.get_logger().info('Color Selector initialized. Click pixels to select colors.')
        self.get_logger().info('Press G/B/Y/P to switch color modes. Press C to clear current mode. Press O to output bounds. Press ESC to exit.')
        
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Apply bilateral filter
            filtered_image = cv2.bilateralFilter(cv_image, 9, sigmaColour, sigmaSpace)

            self.current_image = filtered_image
            
            # Show original filtered image in Color Selector window
            selector_display = filtered_image.copy()
            cv2.imshow('Color Selector', selector_display)
            
            # Create and show masked result for current color only
            result_image = self.create_masked_result(filtered_image)
            cv2.imshow('Color results', result_image)
            
            # Create and show text display window
            text_display = self.create_text_display()
            cv2.imshow('Text Display', text_display)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC key
                self.get_logger().info('Exiting...')
                cv2.destroyAllWindows()
                rclpy.shutdown()
            elif key == ord('g') or key == ord('G'):
                self.current_color = 'G'
                self.get_logger().info('Switched to Green selection mode')
            elif key == ord('b') or key == ord('B'):
                self.current_color = 'B'
                self.get_logger().info('Switched to Blue selection mode')
            elif key == ord('y') or key == ord('Y'):
                self.current_color = 'Y'
                self.get_logger().info('Switched to Yellow selection mode')
            elif key == ord('p') or key == ord('P'):
                self.current_color = 'P'
                self.get_logger().info('Switched to Pink selection mode')
            elif key == ord('c') or key == ord('C'):
                # Clear current color mode
                self.colors[self.current_color] = []
                self.get_logger().info(f'Cleared {self.current_color} color mode')
            elif key == ord('o') or key == ord('O'):
                # Output formatted color bounds
                self.output_color_bounds()
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Get the current image from the window
            current_image = self.current_image.copy()

            if current_image is not None:
                # Get the pixel color at clicked position
                pixel_color = current_image[y, x]
                
                # Convert BGR to HSV for better color matching
                hsv_pixel = cv2.cvtColor(np.uint8([[pixel_color]]), cv2.COLOR_BGR2HSV)[0][0]
                
                # Store the color
                self.colors[self.current_color].append(hsv_pixel)
                
                self.get_logger().info(f'Added {self.current_color} color at ({x}, {y}): HSV{hsv_pixel}')
                self.get_logger().info(f'{self.current_color} colors: {len(self.colors[self.current_color])} points')
                
                # Print bounds if we have enough points
                if len(self.colors[self.current_color]) >= 2:
                    bounds = self.calculate_color_bounds(self.colors[self.current_color])
                    self.get_logger().info(f'{self.current_color} bounds - H: {bounds["h_min"]}-{bounds["h_max"]}, S: {bounds["s_min"]}-{bounds["s_max"]}, V: {bounds["v_min"]}-{bounds["v_max"]}')
    
    def create_masked_result(self, image):
        """Create masked result for current color mode only"""
        # Get current color points
        color_points = self.colors[self.current_color]
        
        # If we don't have enough points, show black image
        if len(color_points) < 2:
            result = np.zeros_like(image)
        else:
            # Convert to HSV
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # Get bounds for current color
            bounds = self.calculate_color_bounds(color_points)
            lower = np.array([bounds['h_min'], bounds['s_min'], bounds['v_min']])
            upper = np.array([bounds['h_max'], bounds['s_max'], bounds['v_max']])
            
            # Mask out everything except pixels in range
            mask = cv2.inRange(hsv_image, lower, upper)
            result = cv2.bitwise_and(image, image, mask=mask)
        
        return result
    
    def create_color_mask(self, hsv_image, color_points):
        """Create mask based on color points"""
        if len(color_points) < 2:
            return np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        
        # Get bounds using the new method
        bounds = self.calculate_color_bounds(color_points)
        h_min, h_max = bounds['h_min'], bounds['h_max']
        s_min, s_max = bounds['s_min'], bounds['s_max']
        v_min, v_max = bounds['v_min'], bounds['v_max']
        
        # Handle HSV wrap-around for hue
        if h_max - h_min > 179:
            # Hue wraps around (e.g., red colors)
            lower1 = np.array([0, s_min, v_min])
            upper1 = np.array([h_max, s_max, v_max])
            lower2 = np.array([h_min, s_min, v_min])
            upper2 = np.array([179, s_max, v_max])
            mask1 = cv2.inRange(hsv_image, lower1, upper1)
            mask2 = cv2.inRange(hsv_image, lower2, upper2)
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            lower = np.array([h_min, s_min, v_min])
            upper = np.array([h_max, s_max, v_max])
            mask = cv2.inRange(hsv_image, lower, upper)
        
        return mask
    
    def get_color_bgr(self, color_name):
        """Get BGR color for overlay"""
        color_map = {
            'G': (0, 255, 0),    # Green
            'B': (255, 0, 0),    # Blue
            'Y': (0, 255, 255),  # Yellow
            'P': (255, 0, 255)   # Purple
        }
        return color_map.get(color_name, (128, 128, 128))
    
    def get_color_name(self, color_code):
        """Get full color name from code"""
        color_name_map = {
            'G': 'green',
            'B': 'blue',
            'Y': 'yellow',
            'P': 'pink'
        }
        return color_name_map.get(color_code, 'unknown')
    
    def calculate_color_bounds(self, color_points):
        """Calculate HSV bounds from color points"""
        if len(color_points) < 2:
            return {'h_min': 0, 'h_max': 179, 's_min': 0, 's_max': 255, 'v_min': 0, 'v_max': 255}
        
        # Calculate range from color points
        h_values = [point[0] for point in color_points]
        s_values = [point[1] for point in color_points]
        v_values = [point[2] for point in color_points]
        
        # Expand range to include all selected points with tolerance
        h_min = max(0, min(h_values) - TOLERANCE_H)
        h_max = min(179, max(h_values) + TOLERANCE_H)
        s_min = max(0, min(s_values) - TOLERANCE_S)
        s_max = min(255, max(s_values) + TOLERANCE_S)
        v_min = max(0, min(v_values) - TOLERANCE_V)
        v_max = min(255, max(v_values) + TOLERANCE_V)
        
        return {
            'h_min': h_min, 'h_max': h_max,
            's_min': s_min, 's_max': s_max,
            'v_min': v_min, 'v_max': v_max
        }

    def output_color_bounds(self):
        """Output color bounds in the desired format"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('COLOR BOUNDS OUTPUT')
        self.get_logger().info('='*60)
        
        print('\ncolours = {')
        
        for color_code, color_points in self.colors.items():
            if len(color_points) >= 2:
                bounds = self.calculate_color_bounds(color_points)
                color_name = self.get_color_name(color_code)
                
                lower_bound = (bounds['h_min'], bounds['s_min'], bounds['v_min'])
                upper_bound = (bounds['h_max'], bounds['s_max'], bounds['v_max'])
                
                print(f'\t"{color_name}":\t\t({lower_bound}, {upper_bound}),')
        
        print('}')
        self.get_logger().info('='*60 + '\n')

    def create_text_display(self):
        """Create a clean text display window with larger, clearer text"""
        # Create a black canvas
        canvas_height = 650
        canvas_width = 800
        text_display = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)
        
        # Title
        cv2.putText(text_display, "Color Selection Status", (50, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
        
        # Current mode
        cv2.putText(text_display, f"Current Mode: {self.current_color}", (50, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        
        # Instructions
        cv2.putText(text_display, "Controls:", (50, 180), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        cv2.putText(text_display, "G/B/Y/P - Switch color modes", (70, 220), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(text_display, "C - Clear current mode", (70, 250), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(text_display, "O - Output bounds", (70, 280), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(text_display, "ESC - Exit", (70, 310), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # Color status section
        cv2.putText(text_display, "Color Status:", (50, 370), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        
        # Display each color's status
        y_start = 410
        for i, (color_name, color_points) in enumerate(self.colors.items()):
            y_pos = y_start + (i * 50)
            count = len(color_points)
            color_bgr = self.get_color_bgr(color_name)
            
            # Color name and count
            cv2.putText(text_display, f"{color_name}: {count} points", (70, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, color_bgr, 2)
            
            # Show bounds if we have enough points
            if count >= 2:
                bounds = self.calculate_color_bounds(color_points)
                bounds_text = f"H:{bounds['h_min']}-{bounds['h_max']} S:{bounds['s_min']}-{bounds['s_max']} V:{bounds['v_min']}-{bounds['v_max']}"
                cv2.putText(text_display, bounds_text, (250, y_pos), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            else:
                cv2.putText(text_display, "Need 2+ points for bounds", (250, y_pos), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
        
        return text_display


def main(args=None):
    rclpy.init(args=args)
    color_selector = ColorSelector()
    
    try:
        rclpy.spin(color_selector)
    except KeyboardInterrupt:
        pass
    finally:
        color_selector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()