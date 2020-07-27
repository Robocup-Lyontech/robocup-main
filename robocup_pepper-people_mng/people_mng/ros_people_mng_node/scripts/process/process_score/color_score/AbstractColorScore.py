__author__ = 'Jacques Saraydaryan'
from abc import ABCMeta, abstractmethod

class AbstractColorScore:
    __metaclass__ = ABCMeta

    @abstractmethod
    def color_score(self, people_color_list, tracked_people, KEY): pass


    @abstractmethod
    def update_color_score_content(self,tracked_people_id_to_remove_list): pass

    def get_max_hsv_color(self, colorList):
        """
        Retrun the max color of a given color list (from k-mean clustering)
        :param colorList: color list to compute
        :return: normaliszed HSV
        """
        hsv = [0,0,0]
        hsv_result = []
        max_value = 0.0
        for color in colorList:
            if color.percentage_of_img > max_value:
                max_value = color.percentage_of_img
                hsv = color.hsv
        # normalisation due to the encoding format
        hsv_result.append(hsv[0] * 360)
        hsv_result.append(hsv[1] * 100)
        hsv_result.append(hsv[2] * 100)
        return hsv_result

    @abstractmethod
    def add_color_to_tracked(self, people_color_list, tracked_people, KEY): pass