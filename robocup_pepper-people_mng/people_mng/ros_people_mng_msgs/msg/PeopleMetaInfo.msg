#id of the detected 2d people
string id
#id of the face detection
string label_id
#score of the current label
float32 label_score
# given by gossip node
string[] handPosture
# given by gossip node could be standing, sitting, lying
string posture
# given by gossid node, distance estimation only through 2d image
float32 distanceEval
geometry_msgs/Pose   pose #

# given by color detection node, color name of the shirt
string shirt_color_name
# given by color detection node, trouser name of the shirt
string trouser_color_name

# details information including global bounding box, tshirt and trouser colors and bounding box data
ros_people_mng_msgs/PeopleMetaInfoDetails details