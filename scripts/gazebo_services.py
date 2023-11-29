import rospy
from gazebo_msgs.srv import GetWorldProperties, GetModelProperties, GetModelState, DeleteModel
import math

# Provides functions to find the nearest rubbish in the gazebo world and remove it

RUBBSIH_MODEL_IDENTIFIERS = "green_square" # TEMP - The model of rubbish
RECYCLING_MODEL_IDENTIFIERS = "red_square"
ROBOT_MODEL_NAME = "p3dx" # Model name for the robot, used to get the nearest models to the robot

def get_models(): # Gets all models in the world
    service = 'gazebo/get_world_properties'
    rospy.wait_for_service(service)
    try:
        model_list = rospy.ServiceProxy(service, GetWorldProperties)
        response: str = str(model_list())
        models = list(filter(lambda x: '-' in x, response.split("\n")))
        models = list(map(lambda x: x.replace('-', '').strip(), models))
        return models
    except rospy.ServiceException as e:
        print("error occured", e)


def get_closest_model(model_names: [str]): # Gets the closest model
    service = 'gazebo/get_model_state'
    rospy.wait_for_service(service)
    try:
        model_info = rospy.ServiceProxy(service, GetModelState)
        responses = []
        for m in model_names:
            responses.append((model_info(m, ROBOT_MODEL_NAME)))
        model_positions = [[info.pose.position.x, info.pose.position.y] for info in responses]
        distances = [math.sqrt(pos[0]**2 + pos[1]**2) for pos in model_positions]
        if len(distances) == 0:
            return ""
        closest = model_names[distances.index(min(distances))]
        return closest
    except rospy.ServiceException as e:
        print("error occured", e)

def delete_model(model_name): # Deletes the model with the specified name
    service = 'gazebo/delete_model'
    rospy.wait_for_service(service)
    try:
        model_delete = rospy.ServiceProxy(service, DeleteModel)
        model_delete(model_name)
    except rospy.ServiceException as e:
        print("error occured", e)

def remove_nearest_rubbish():
    world_models = get_models()
    rubbish_models = [m for m in world_models if RUBBSIH_MODEL_IDENTIFIERS in m or RECYCLING_MODEL_IDENTIFIERS in m]
    closest_model = get_closest_model(rubbish_models)
    delete_model(closest_model)

remove_nearest_rubbish()