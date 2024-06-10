import droneapi
class Lidar2D:
    def __init__(self,obj_coordinates,obj_size,drone:droneapi.Drone):
        # Initialize a list of 360 elements with None to represent uninitialized distances
        self.distances = [None] * 360
        self.loc = obj_coordinates
        self.obj_size = obj_size
        self.drone = drone
        self.object_ahead = False




    def update_distance(self, angle, distance):

        """
        Updates the distance at a specific angle.

        :param angle: The angle at which to update the distance (0-359).
        :param distance: The distance to set at the given angle.
        """
        if 0 <= angle < 360:
            self.distances[angle] = distance
        else:
            raise ValueError("Angle must be between 0 and 359 degrees.")
    def set_object(self,direction):
        if direction == 'Front':
            for i in range(self.drone.get_heading()-30,self.drone.get_heading()+30):
                self.distances[i] = 5

    def check_object(self,direction):
        if direction == 'Front':
            for i in range(self.drone.get_heading()-30,self.drone.get_heading()+30):
                if self.distances[i] > 1:
                    self.object_ahead = True
                    break





    def get_distance(self, angle):
        """
        Retrieves the distance at a specific angle.

        :param angle: The angle for which to get the distance (0-359).
        :return: The distance at the given angle.
        """
        if 0 <= angle < 360:
            return self.distances[angle]
        else:
            raise ValueError("Angle must be between 0 and 359 degrees.")

    def clear_distances(self):
        """
        Resets all distances to None.
        """
        self.distances = [None] * 360

    def __str__(self):
        """
        Returns a string representation of the LiDAR distances.
        """
        return str(self.distances)


# Example usage:
lidar = Lidar2D()

# Update some distances
lidar.update_distance(0, 5.0)
lidar.update_distance(90, 3.2)
lidar.update_distance(180, 7.5)
lidar.update_distance(270, 2.1)

# Retrieve a distance
print(f"Distance at 90 degrees: {lidar.get_distance(90)}")

# Print all distances
print(lidar)
