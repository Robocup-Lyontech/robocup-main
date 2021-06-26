#!/usr/bin/env python  

__author__ ='Raphael Leber'


import rospy 
import actionlib
from std_msgs.msg import String, Int16, Empty
from robocup_launcher.srv import MessageParserSrv, MessageParserSrvResponse
from robocup_launcher.srv import LabelDepositSrv, LabelDepositSrvResponse



class MessageParser():

    def __init__(self):
        rospy.init_node('message_parser_node', anonymous=False)

        # --------------
        # Declare topics
        # --------------
        # * Suscribers
        rospy.Subscriber("/message", String, self.message)
        # * Publishers
        self.pubPerson = rospy.Publisher('/message/person', String, queue_size=1)
        self.pubObject = rospy.Publisher('/message/object', String, queue_size=1)
        self.pubObjectNum = rospy.Publisher('/message/object_num', Int16, queue_size=1)
        self.pubObjectDarknet = rospy.Publisher('/message/object_darknet', String, queue_size=1)

        # --------------
        # Declare Services
        # --------------
        self.service = rospy.Service('message_parser', MessageParserSrv, self.serve_parsed_message)
        self.serviceDeposit = rospy.Service('get_deposit', LabelDepositSrv, self.get_deposit_clbk)

        # Declare attributes
        self.person = "pending"
        self.object = "pending"
        self.object_num = -1
        self.object_darknet = "pending"

        self.load_dictionnaries()

        rospy.loginfo("MessageParser init")

        rate = rospy.Rate(1) # 1hz
        while not rospy.is_shutdown():
            self.pubPerson.publish(self.person)
            self.pubObject.publish(self.object)
            self.pubObjectNum.publish(self.object_num)
            self.pubObjectDarknet.publish(self.object_darknet)
            rate.sleep()        

        rospy.spin()


    # /message Topic callback
    def message(self, msg):

        if("left" in msg.data.lower()):
            self.person = "left"
        elif("right" in msg.data.lower()):
            self.person = "right"
        else:
            self.person = "undefined"

        num, name = self.match_object(msg.data)

        if( num == 0 ):
            self.object = "undefined"
            self.object_darknet = "undefined"
        else:
            self.object = name

        self.object_num = num

        if( num > 0 ):
            self.object_darknet = self.ycb_num_to_darknet_label(num)

    # Service callback
    def serve_parsed_message(self, req):
        
        mp = MessageParserSrvResponse()

        mp.person = self.person
        mp.object = self.object
        mp.object_num = self.object_num
        mp.object_darknet = self.object_darknet

        return mp


    # check if there is an object name of the ycb dataset is in the /message request
    def match_object(self, smsg):

        match = False
        
        for num, names in self.ycb_number_to_ycb_names.items():
            if( len(names) > 0 ):
                for name in names:
                    if( name.lower() in smsg.lower() ):
                        return num, name
                    if( name.replace('_',' ').lower() in smsg.lower() ):
                        return num, name                        
        
        return 0, "undefined"

    
    def ycb_num_to_darknet_label(self, num):
        # list out keys and values separately
        key_list = list(self.darknet_label_to_ycb_number.keys())
        val_list = list(self.darknet_label_to_ycb_number.values())
        
        # print key with val num
        try:
            position = val_list.index(num)
            return key_list[position] 
        except:
            return "undefined"

               
    def get_deposit_clbk(self, req):
        ld = LabelDepositSrvResponse()
        ld.deposit = self.get_deposit(req.label)
        return ld


    # Get list of deposit locations from a darknet label
    def get_deposit(self, darknet_label):
        ycb_num = self.darknet_label_to_ycb_number[darknet_label]

        if ycb_num in self.ycbNumObj_deposit.keys():
            deposit = self.ycbNumObj_deposit[ycb_num]["Deposit"]
        else:
            deposit = []

        return deposit



    # Load YCB label dictionnary
    def load_dictionnaries(self):

        self.ycb_number_to_ycb_names =    \
            {
                1:["chips_can"],
                2:["master_chef_can", "coffee"],
                3:["cracker_box", "Cheez-it"],
                4:["sugar_box"],
                5:["tomato_soup_can"],
                6:["mustard_bottle"],
                7:["tuna_fish_can"],
                8:["pudding_box"],
                9:["gelatin_box"],
                10:["potted_meat_can"],
                11:["banana"],
                12:["strawberry"],
                13:["apple"],
                14:["lemon"],
                15:["peach"],
                16:["pear"],
                17:["orange"],
                18:["plum"],
                19:["pitcher_base"],
                20:[],
                21:["bleach_cleanser"],
                22:["windex_bottle"],
                23:["wine_glass"],
                24:["bowl"],
                25:["mug"],
                26:["sponge"],
                27:["skillet"],
                28:["skillet_lid"],
                29:["plate"],
                30:["fork"],
                31:["spoon"],
                32:["knife"],
                33:["spatula"],
                34:[],
                35:["power_drill"],
                36:["wood_block"],
                37:["scissors"],
                38:["padlock"],
                39:["key"],
                40:["large_marker"],
                41:["small_marker"],
                42:["adjustable_wrench"],
                43:["phillips_screwdriver"],
                44:["flat_screwdriver"],
                45:[],
                46:["plastic_bolt"],
                47:["plastic_nut"],
                48:["hammer"],
                49:["small_clamp"],
                50:["medium_clamp"],
                51:["large_clamp"],
                52:["extra_large_clamp"],
                53:["mini_soccer_ball"],
                54:["softball"],
                55:["baseball"],
                56:["tennis_ball"],
                57:["racquetball"],
                58:["golf_ball"],
                59:["chain"],
                60:[],
                61:["foam_brick"],
                62:["dice"],
                63:["marbles", "a_marbles", "b_marbles", "c_marbles", "d_marbles", "e_marbles", "f_marbles"],
                62:[],
                63:[],
                64:[],
                65:["cups", "a_cups", "b_cups", "c_cups", "d_cups", "e_cups","f_cups","g_cups","h_cups","i_cups","j_cups"],
                66:[],
                67:[],
                68:[],
                69:[],
                70:["colored_wood_blocks", "a_colored_wood_blocks", "b_colored_wood_blocks"],
                71:["nine_hole_peg_test"],
                72:["toy_airplane", "a_toy_airplane", "b_toy_airplane", "c_toy_airplane", "d_toy_airplane", "e_toy_airplane", "f_toy_airplane", "g_toy_airplane", "h_toy_airplane", "i_toy_airplane","j_toy_airplane", "k_toy_airplane"],
                73:["a_lego_duplo","b_lego_duplo", "c_lego_duplo", "d_lego_duplo", "e_lego_duplo", "f_lego_duplo", "g_lego_duplo", "h_lego_duplo", "i_lego_duplo", "j_lego_duplo", "k_lego_duplo", "l_lego_duplo", "m_lego_duplo"],
                74:[],
                75:[],
                76:["timer"],
                77:["rubiks_cube"]
            }


        self.darknet_label_to_ycb_number = \
            {
                "cracker"               : 3 , 
                "sugar"                 : 4 ,
                "pudding"               : 8 ,
                "gelatin"               : 9 ,
                "pottedmeat"            : 10,
                "coffee"                : 2 ,
                "tuna"                  : 7 ,
                "chips"                 : 1 ,
                "mustard"               : 6 ,
                "tomatosoup"            : 5 ,
                "banana"                : 11,
                "strawberry"            : 12,
                "apple"                 : 13,
                "lemon"                 : 14,
                "peach"                 : 15,
                "pear"                  : 16,
                "orange"                : 17,
                "plum"                  : 18,
                "windex"                : 22,
                "bleach"                : 21,
                "pitcher"               : 19,
                "plate"                 : 29,
                "bowl"                  : 24,
                "fork"                  : 30,
                "spoon"                 : 26,
                "spatula"               : 33,
                "wineglass"             : 23,
                "cup"                   : 65,
                "largemarker"           : 40,
                "smallmarker"           : 41,
                "padlocks"              : 38,
                "bolt"                  : 46,
                "nut"                   : 47,
                "clamp"                 : 49,
                "soccerball"            : 53,
                "baseball"              : 55,
                "tennisball"            : 56,
                "golfball"              : 58,
                "foambrick"             : 61,
                "dice"                  : 62,
                "rope"                  : -1,
                "chain"                 : 59,
                "rubikscube"            : 77,
                "coloredwoodblock"      : 70,
                "peghole"               : 71,
                "timer"                 : 76,
                "airplane"              : 72,
                "tshirt"                : -1,
                "magazine"              : -1,
                "creditcard"            : -1,
                "legoduplo"             : 73,
                "sponge"                : 26,
                "coloredwoodblockpot"   : 70,
                "softball"              : 54,
                "racquetball"           : 57,
                "marbles"               : 63,
                "mug"                   : 25
            }


        self.ycbNumObj_deposit =    \
            {
                1:{"Category":"food", "Deposit": ["Tray_A", "Tray_B"], "Place":["Long_Table_A"]},
                2:{"Category":"food", "Deposit": ["Tray_A", "Tray_B"], "Place":["Long_Table_A"]},
                3:{"Category":"food", "Deposit": ["Tray_A", "Tray_B"], "Place":["Long_Table_A"]},
                4:{"Category":"food", "Deposit": ["Tray_A", "Tray_B"], "Place":["Long_Table_A"]},
                5:{"Category":"food", "Deposit": ["Tray_A", "Tray_B"], "Place":["Long_Table_A"]},
                6:{"Category":"food", "Deposit": ["Tray_A", "Tray_B"], "Place":["Long_Table_A"]},
                7:{"Category":"food", "Deposit": ["Tray_A", "Tray_B"], "Place":["Long_Table_A"]},
                8:{"Category":"food", "Deposit": ["Tray_A", "Tray_B"], "Place":["Long_Table_A"]},
                9:{"Category":"food", "Deposit": ["Tray_A", "Tray_B"], "Place":["Long_Table_A"]},
                10:{"Category":"food", "Deposit": ["Tray_A", "Tray_B"], "Place":["Long_Table_A"]},
                11:{"Category":"food", "Deposit": ["Tray_A", "Tray_B"], "Place":["Long_Table_A"]},
                12:{"Category":"food", "Deposit": ["Tray_A", "Tray_B"], "Place":["Long_Table_A"]},
                13:{"Category":"food", "Deposit": ["Tray_A", "Tray_B"], "Place":["Long_Table_A"]},
                14:{"Category":"food", "Deposit": ["Tray_A", "Tray_B"], "Place":["Long_Table_A"]},
                15:{"Category":"food", "Deposit": ["Tray_A", "Tray_B"], "Place":["Long_Table_A"]},
                16:{"Category":"food", "Deposit": ["Tray_A", "Tray_B"], "Place":["Long_Table_A"]},
                17:{"Category":"food", "Deposit": ["Tray_A", "Tray_B"], "Place":["Long_Table_A"]},
                18:{"Category":"food", "Deposit": ["Tray_A", "Tray_B"], "Place":["Long_Table_A"]},

                19:{"Category":"kitchenitem", "Deposit": ["Container_A"], "Place":["Long_Table_A"]},
                20:{"Category":"kitchenitem", "Deposit": ["Container_A"], "Place":["Long_Table_A"]},
                21:{"Category":"kitchenitem", "Deposit": ["Container_A"], "Place":["Long_Table_A"]},
                22:{"Category":"kitchenitem", "Deposit": ["Container_A"], "Place":["Long_Table_A"]},
                23:{"Category":"kitchenitem", "Deposit": ["Container_A"], "Place":["Long_Table_A"]},
                24:{"Category":"kitchenitem", "Deposit": ["Container_A"], "Place":["Long_Table_A"]},
                25:{"Category":"kitchenitem", "Deposit": ["Container_A"], "Place":["Long_Table_A"]},
                26:{"Category":"kitchenitem", "Deposit": ["Container_A"], "Place":["Long_Table_A"]},
                27:{"Category":"kitchenitem", "Deposit": ["Container_A"], "Place":["Long_Table_A"]},
                28:{"Category":"kitchenitem", "Deposit": ["Container_A"], "Place":["Long_Table_A"]},
                29:{"Category":"kitchenitem", "Deposit": ["Container_A"], "Place":["Long_Table_A"]},
                30:{"Category":"kitchenitem", "Deposit": ["Container_A"], "Place":["Long_Table_A"]},
                31:{"Category":"kitchenitem", "Deposit": ["Container_A"], "Place":["Long_Table_A"]},
                32:{"Category":"kitchenitem", "Deposit": ["Container_A"], "Place":["Long_Table_A"]},
                33:{"Category":"kitchenitem", "Deposit": ["Container_A"], "Place":["Long_Table_A"]},


                38:{"Category":"tool", "Deposit": ["Drawer_top", "Drawer_bottom"], "Place":["Drawer"]},


                40:{"Category":"tool", "Deposit": ["Drawer_top", "Drawer_bottom"], "Place":["Drawer"]},
                41:{"Category":"tool", "Deposit": ["Drawer_top", "Drawer_bottom"], "Place":["Drawer"]},


                46:{"Category":"tool", "Deposit": ["Drawer_top", "Drawer_bottom"], "Place":["Drawer"]},
                47:{"Category":"tool", "Deposit": ["Drawer_top", "Drawer_bottom"], "Place":["Drawer"]},
                48:{"Category":"tool", "Deposit": ["Drawer_top", "Drawer_bottom"], "Place":["Drawer"]},
                49:{"Category":"tool", "Deposit": ["Drawer_top", "Drawer_bottom"], "Place":["Drawer"]},
                50:{"Category":"tool", "Deposit": ["Drawer_top", "Drawer_bottom"], "Place":["Drawer"]},
                51:{"Category":"tool", "Deposit": ["Drawer_top", "Drawer_bottom"], "Place":["Drawer"]},
                52:{"Category":"tool", "Deposit": ["Drawer_top", "Drawer_bottom"], "Place":["Drawer"]},
                
                53:{"Category":"shapeitem", "Deposit": ["Drawer_left"], "Place":["Drawer"]},
                54:{"Category":"shapeitem", "Deposit": ["Drawer_left"], "Place":["Drawer"]},
                55:{"Category":"shapeitem", "Deposit": ["Drawer_left"], "Place":["Drawer"]},
                56:{"Category":"shapeitem", "Deposit": ["Drawer_left"], "Place":["Drawer"]},
                57:{"Category":"shapeitem", "Deposit": ["Drawer_left"], "Place":["Drawer"]},
                58:{"Category":"shapeitem", "Deposit": ["Drawer_left"], "Place":["Drawer"]},
                59:{"Category":"shapeitem", "Deposit": ["Drawer_left"], "Place":["Drawer"]},
                60:{"Category":"shapeitem", "Deposit": ["Drawer_left"], "Place":["Drawer"]},
                61:{"Category":"shapeitem", "Deposit": ["Drawer_left"], "Place":["Drawer"]},
                62:{"Category":"shapeitem", "Deposit": ["Drawer_left"], "Place":["Drawer"]},
                63:{"Category":"shapeitem", "Deposit": ["Drawer_left"], "Place":["Drawer"]},
                62:{"Category":"shapeitem", "Deposit": ["Drawer_left"], "Place":["Drawer"]},
                63:{"Category":"shapeitem", "Deposit": ["Drawer_left"], "Place":["Drawer"]},
                64:{"Category":"shapeitem", "Deposit": ["Drawer_left"], "Place":["Drawer"]},
                65:{"Category":"shapeitem", "Deposit": ["Drawer_left"], "Place":["Drawer"]},


                70:{"Category":"taskitem", "Deposit": ["Bin_A"], "Place":["Bin_A"]},
                71:{"Category":"taskitem", "Deposit": ["Bin_A"], "Place":["Bin_A"]},
                72:{"Category":"taskitem", "Deposit": ["Bin_A"], "Place":["Bin_A"]},
                73:{"Category":"taskitem", "Deposit": ["Bin_A"], "Place":["Bin_A"]},


                76:{"Category":"taskitem", "Deposit": ["Bin_A"], "Place":["Bin_A"]},

                77:{"Category":"taskitem", "Deposit": ["Bin_A"], "Place":["Bin_A"]}
            }      



if __name__ == '__main__':
    mp = MessageParser()