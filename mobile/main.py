#!/usr/bin/env python
from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.gridlayout import GridLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.graphics import Color, Rectangle, Line, Ellipse
from kivy.clock import Clock
from kivy.properties import NumericProperty, ListProperty, ObjectProperty, BooleanProperty

import rospy
from std_msgs.msg import String
from dijkstra import Graph, dijkstra_shortest_path

class GridCell(Widget):
    row = NumericProperty(0)
    col = NumericProperty(0)
    is_obstacle = BooleanProperty(False)
    is_start = BooleanProperty(False)
    is_goal = BooleanProperty(False)
    is_path = BooleanProperty(False)
    
    def on_touch_down(self, touch):
        if self.collide_point(*touch.pos):
            print(f"Cell touched at row={self.row}, col={self.col}")
            if self.parent.start_mode:
                print(f"Setting start position to ({self.row}, {self.col})")
                self.parent.clear_start()
                self.is_start = True
                self.parent.start_pos = (self.row, self.col)
                self.parent.start_mode = False
            elif self.parent.goal_mode:
                print(f"Setting goal position to ({self.row}, {self.col})")
                self.parent.clear_goal()
                self.is_goal = True
                self.parent.goal_pos = (self.row, self.col)
                self.parent.goal_mode = False
            else:
                self.is_obstacle = not self.is_obstacle
                print(f"Toggle obstacle at ({self.row}, {self.col}) to {self.is_obstacle}")
                self.parent.update_graph()
            return True
        return super(GridCell, self).on_touch_down(touch)
    
    def on_is_obstacle(self, instance, value):
        print(f"Updating cell appearance at ({self.row}, {self.col})")
        self.canvas.clear()
        with self.canvas:
            if self.is_start:
                print(f"Drawing start cell at ({self.row}, {self.col})")
                Color(0, 1, 0, 1)  # Green for start
            elif self.is_goal:
                print(f"Drawing goal cell at ({self.row}, {self.col})")
                Color(1, 0, 0, 1)  # Red for goal
            elif self.is_path:
                print(f"Drawing path cell at ({self.row}, {self.col})")
                Color(0, 0, 1, 1)  # Blue for path
            elif self.is_obstacle:
                print(f"Drawing obstacle cell at ({self.row}, {self.col})")
                Color(0.2, 0.2, 0.2, 1)  # Dark gray for obstacles
            else:
                print(f"Drawing empty cell at ({self.row}, {self.col})")
                Color(0.9, 0.9, 0.9, 1)  # Light gray for empty cells
            Rectangle(pos=self.pos, size=self.size)
    
    def on_is_start(self, instance, value):
        print(f"Start status changed at ({self.row}, {self.col}) to {value}")
        self.on_is_obstacle(instance, value)
    
    def on_is_goal(self, instance, value):
        print(f"Goal status changed at ({self.row}, {self.col}) to {value}")
        self.on_is_obstacle(instance, value)
    
    def on_is_path(self, instance, value):
        print(f"Path status changed at ({self.row}, {self.col}) to {value}")
        self.on_is_obstacle(instance, value)

class NavigationGrid(GridLayout):
    rows_count = NumericProperty(10)
    cols_count = NumericProperty(10)
    start_pos = ListProperty([0, 0])
    goal_pos = ListProperty([9, 9])
    start_mode = BooleanProperty(False)
    goal_mode = BooleanProperty(False)
    graph = ObjectProperty(None)
    path = ListProperty([])
    
    def __init__(self, **kwargs):
        print("Initializing NavigationGrid")
        super(NavigationGrid, self).__init__(**kwargs)
        self.rows = self.rows_count
        self.cols = self.cols_count
        self.cells = {}
        self.graph = Graph()
        
        print(f"Creating grid with {self.rows_count} rows and {self.cols_count} columns")
        for row in range(self.rows_count):
            for col in range(self.cols_count):
                print(f"Adding cell at ({row}, {col})")
                cell = GridCell(row=row, col=col)
                self.add_widget(cell)
                self.cells[(row, col)] = cell
        
        print(f"Setting initial start position to {tuple(self.start_pos)}")
        self.cells[tuple(self.start_pos)].is_start = True
        print(f"Setting initial goal position to {tuple(self.goal_pos)}")
        self.cells[tuple(self.goal_pos)].is_goal = True
        self.update_graph()
        print("NavigationGrid initialization complete")
    
    def clear_start(self):
        print(f"Clearing start position at {tuple(self.start_pos)}")
        if tuple(self.start_pos) in self.cells:
            self.cells[tuple(self.start_pos)].is_start = False
    
    def clear_goal(self):
        print(f"Clearing goal position at {tuple(self.goal_pos)}")
        if tuple(self.goal_pos) in self.cells:
            self.cells[tuple(self.goal_pos)].is_goal = False
    
    def clear_path(self):
        print(f"Clearing path with {len(self.path)} cells")
        for pos in self.path:
            if pos != tuple(self.start_pos) and pos != tuple(self.goal_pos):
                print(f"Clearing path cell at {pos}")
                self.cells[pos].is_path = False
        self.path = []
        print("Path cleared")
    
    def update_graph(self):
        print("Updating graph with current grid state")
        self.graph = Graph()
        edge_count = 0
        
        for row in range(self.rows_count):
            for col in range(self.cols_count):
                if not self.cells[(row, col)].is_obstacle:
                    # Add edges to adjacent non-obstacle cells
                    for dr, dc in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                        nr, nc = row + dr, col + dc
                        if 0 <= nr < self.rows_count and 0 <= nc < self.cols_count:
                            if not self.cells[(nr, nc)].is_obstacle:
                                self.graph.add_edge((row, col), (nr, nc), 1)
                                edge_count += 1
        
        print(f"Graph updated with {edge_count} edges")
    
    def calculate_path(self):
        print("Calculating shortest path")
        self.clear_path()
        start = tuple(self.start_pos)
        goal = tuple(self.goal_pos)
        
        print(f"Finding path from {start} to {goal}")
        
        if start == goal:
            print("Start and goal are the same, no path needed")
            return
        
        print("Running Dijkstra's algorithm")
        distances, previous = dijkstra_shortest_path(self.graph, start)
        
        if goal not in previous:
            print("No path found to goal")
            return
        
        # Reconstruct path
        print("Reconstructing path from Dijkstra results")
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = previous[current]
        path.append(start)
        path.reverse()
        
        self.path = path
        print(f"Path found with {len(path)} steps: {path}")
        
        # Mark path cells
        for pos in path:
            if pos != start and pos != goal:
                print(f"Marking path cell at {pos}")
                self.cells[pos].is_path = True
        
        print("Path calculation complete")
        return path

class ControlPanel(BoxLayout):
    grid = ObjectProperty(None)
    ros_publisher = ObjectProperty(None)
    
    def set_start(self):
        print("Set start mode activated")
        self.grid.start_mode = True
        self.grid.goal_mode = False
    
    def set_goal(self):
        print("Set goal mode activated")
        self.grid.start_mode = False
        self.grid.goal_mode = True
    
    def find_path(self):
        print("Finding path button pressed")
        path = self.grid.calculate_path()
        if path and len(path) > 1:
            print(f"Path found, sending {len(path)-1} commands")
            self.send_path_commands(path)
        else:
            print("No valid path to send commands for")
    
    def clear_grid(self):
        print("Clearing grid")
        for cell in self.grid.cells.values():
            cell.is_obstacle = False
            cell.is_path = False
        self.grid.update_graph()
        print("Grid cleared")
    
    def send_path_commands(self, path):
        print(f"Preparing to send path commands for path: {path}")
        if not self.ros_publisher:
            print("ERROR: ROS publisher not initialized")
            return
        
        commands = []
        for i in range(len(path) - 1):
            current = path[i]
            next_pos = path[i + 1]
            
            # Determine direction
            dr = next_pos[0] - current[0]
            dc = next_pos[1] - current[1]
            
            if dr == 1:
                commands.append("DOWN")
            elif dr == -1:
                commands.append("UP")
            elif dc == 1:
                commands.append("RIGHT")
            elif dc == -1:
                commands.append("LEFT")
        
        print(f"Generated {len(commands)} commands: {commands}")
        
        # Send commands to ROS
        for i, cmd in enumerate(commands):
            print(f"Sending command {i+1}/{len(commands)}: {cmd}")
            self.ros_publisher.publish(cmd)
            rospy.sleep(0.5)  # Small delay between commands
        
        print("All commands sent")

class NavigationApp(App):
    def build(self):
        print("Building NavigationApp")
        # Initialize ROS node
        print("Initializing ROS node")
        rospy.init_node('kivy_navigation', anonymous=True)
        pub = rospy.Publisher('robot_commands', String, queue_size=10)
        print("ROS publisher created for topic 'robot_commands'")
        
        # Create main layout
        print("Creating main layout")
        main_layout = BoxLayout(orientation='vertical')
        
        # Create grid
        print("Creating navigation grid")
        grid = NavigationGrid(size_hint=(1, 0.8))
        
        # Create control panel
        print("Creating control panel")
        control_panel = ControlPanel(orientation='horizontal', size_hint=(1, 0.2))
        control_panel.grid = grid
        control_panel.ros_publisher = pub
        
        # Add buttons to control panel
        print("Adding buttons to control panel")
        start_btn = Button(text='Set Start')
        start_btn.bind(on_press=lambda x: control_panel.set_start())
        
        goal_btn = Button(text='Set Goal')
        goal_btn.bind(on_press=lambda x: control_panel.set_goal())
        
        find_path_btn = Button(text='Find Path')
        find_path_btn.bind(on_press=lambda x: control_panel.find_path())
        
        clear_btn = Button(text='Clear Grid')
        clear_btn.bind(on_press=lambda x: control_panel.clear_grid())
        
        control_panel.add_widget(start_btn)
        control_panel.add_widget(goal_btn)
        control_panel.add_widget(find_path_btn)
        control_panel.add_widget(clear_btn)
        
        # Add widgets to main layout
        print("Adding grid and control panel to main layout")
        main_layout.add_widget(grid)
        main_layout.add_widget(control_panel)
        
        print("NavigationApp build complete")
        return main_layout

if __name__ == '__main__':
    print("Starting NavigationApp")
    try:
        NavigationApp().run()
    except rospy.ROSInterruptException:
        print("ROS interrupted, shutting down")
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        print("NavigationApp terminated")

