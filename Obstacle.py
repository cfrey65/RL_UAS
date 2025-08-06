class Obstacle:
    def __init__(self, relative_name, pos, scale, color, ori = None):
        """
        Args:
            relative_name (str) : The relative file name for loading the obstacle
            pos (np.array) : A numpy array containting the x, y, z coordinates for the obstacles starting position
            scale (float) : A float that determines the how the object should be scaled when rendered
            color (tuple) : A tuple containing the rgba values for coloring the object
        """
        self.relative_name = relative_name
        self.pos = pos
        self.scale = scale
        self.color = color
        self.ori = ori