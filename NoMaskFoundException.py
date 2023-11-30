class NoMaskFound(Exception):
    """
    Exception raised when no mask is found in the processMasks function.
    """

    def __init__(self, className, message="No mask found for object"):
        self.message = f"{message} '{className}'. Using the bounding box instead."
        super().__init__(self.message)