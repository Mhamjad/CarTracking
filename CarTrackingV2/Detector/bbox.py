from shapely.geometry import Polygon, LineString


class BBox(object):
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.width = x2 - x1
        self.height = y2 - y1
        if y2 > y1:
            lower_side_y = y2
        else:
            lower_side_y = y1
        self.lower_side = LineString([[x1, lower_side_y], [x2, lower_side_y]])

    def __str__(self):
        return "x1: {0}, y1: {1}, x2: {2}, y2: {3},".format(self.x1, self.y1, self.x2, self.y2)

    def get_centre(self):
        return (int(min(self.x1, self.x2) + (self.width / 2)), int(min(self.y1, self.y2) + (self.height / 2)))

    def check_in_frame_boundary(self, boundaries):
        if self.x1 > boundaries.x1 and self.x2 < boundaries.x2 and self.y1 > boundaries.y1 and self.y2 < boundaries.y2:
            return True
        else:
            return False

    def check_if_below_line(self, m, c):
        if -self.y1 < ((m * self.x1) + c):
            return True

    def check_if_frame_intersects(self, bbox):
        return (abs(self.x1 - bbox.x1) * 2 < (self.width + bbox.width)) and (abs(self.y1 - bbox.y1) * 2 < (self.height + bbox.height))

    def check_if_any_frame_intersects(self, points):
        flag = False
        for point in points:
            if point["active"] == True and self.check_if_frame_intersects(point["bbox"]):
                return True
        # for point in points:
        #     if point["bbox"].check_if_frame_intersects(self)):
        #         return True

    def check_if_any_point_in_frame(self, points):
        flag = False
        for point in points:
            if point["active"] == True and self.check_if_point_in_frame(point["centroid"]):
                return True
        for point in points:
            if point["bbox"].check_if_point_in_frame((self.x1 + int(self.width / 2), self.y1 + int(self.width / 2))):
                return True

    def check_if_point_in_frame(self, point):
        if self.x2 > point[0] > self.x1 and self.y2 > point[1] > self.y1:
            return True

    def update(self, dx, dy):
        # refactor to deal with change in angle
        self.x1 += dx
        self.y1 += dy
        self.x2 += dx
        self.y2 += dy
        self.width = self.x2 - self.x1
        self.height = self.y2 - self.y1

    def get_area(self):
        return float((self.x2-self.x1+1)*(self.y2-self.y1+1))

    def get_iou(self, box_1, box_2):
        x_tl_inter = max(box_1.x1, box_2.x1) # x of TL corner of intersection box
        y_tl_inter = max(box_1.y1, box_2.y1) # y of TL corner of intersection box
        x_br_inter = min(box_1.x2, box_2.x2) # x of BR corner of intersection box
        y_br_inter = min(box_1.y2, box_2.y2) # y of BR corner of intersection box

        # compute the area of intersection rectangle
        intersection_area = BBox(x_tl_inter, y_tl_inter, x_br_inter, y_br_inter).get_area()

        # Compute Union
        union_area = float(box_1.get_area() + box_2.get_area() - intersection_area)

        iou = float(intersection_area/union_area)

        # return the intersection over union value
        return iou

    def lower_side_intersects(self, poly):
        if poly.intersects(self.lower_side):
            return True
        else:
            return False
