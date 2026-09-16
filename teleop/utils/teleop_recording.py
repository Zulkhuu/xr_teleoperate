"""Build episode payloads without changing the existing recording schema."""

import logging_mp

logger_mp = logging_mp.getLogger(__name__)


def build_recording_payload(
        camera_config,
        head_img,
        left_wrist_img,
        right_wrist_img,
        left_arm_state,
        right_arm_state,
        left_arm_action,
        right_arm_action,
        left_ee_state,
        right_ee_state,
        left_hand_action,
        right_hand_action,
        current_body_state,
        current_body_action):
    colors = {}
    depths = {}
    if camera_config['head_camera']['binocular']:
        if head_img is not None:
            colors[f"color_{0}"] = head_img.bgr[:, :camera_config['head_camera']['image_shape'][1]//2]
            colors[f"color_{1}"] = head_img.bgr[:, camera_config['head_camera']['image_shape'][1]//2:]
        else:
            logger_mp.warning("Head image is None!")
        if camera_config['left_wrist_camera']['enable_zmq']:
            if left_wrist_img is not None:
                colors[f"color_{2}"] = left_wrist_img.bgr
            else:
                logger_mp.warning("Left wrist image is None!")
        if camera_config['right_wrist_camera']['enable_zmq']:
            if right_wrist_img is not None:
                colors[f"color_{3}"] = right_wrist_img.bgr
            else:
                logger_mp.warning("Right wrist image is None!")
    else:
        if head_img is not None:
            colors[f"color_{0}"] = head_img.bgr
        else:
            logger_mp.warning("Head image is None!")
        if camera_config['left_wrist_camera']['enable_zmq']:
            if left_wrist_img is not None:
                colors[f"color_{1}"] = left_wrist_img.bgr
            else:
                logger_mp.warning("Left wrist image is None!")
        if camera_config['right_wrist_camera']['enable_zmq']:
            if right_wrist_img is not None:
                colors[f"color_{2}"] = right_wrist_img.bgr
            else:
                logger_mp.warning("Right wrist image is None!")
    states = {
        "left_arm": {                                                                    
            "qpos":   left_arm_state.tolist(),    # numpy.array -> list
            "qvel":   [],                          
            "torque": [],                        
        }, 
        "right_arm": {                                                                    
            "qpos":   right_arm_state.tolist(),       
            "qvel":   [],                          
            "torque": [],                         
        },                        
        "left_ee": {                                                                    
            "qpos":   left_ee_state,           
            "qvel":   [],                           
            "torque": [],                          
        }, 
        "right_ee": {                                                                    
            "qpos":   right_ee_state,       
            "qvel":   [],                           
            "torque": [],  
        }, 
        "body": {
            "qpos": current_body_state,
        }, 
    }
    actions = {
        "left_arm": {                                   
            "qpos":   left_arm_action.tolist(),       
            "qvel":   [],       
            "torque": [],      
        }, 
        "right_arm": {                                   
            "qpos":   right_arm_action.tolist(),       
            "qvel":   [],       
            "torque": [],       
        },                         
        "left_ee": {                                   
            "qpos":   left_hand_action,       
            "qvel":   [],       
            "torque": [],       
        }, 
        "right_ee": {                                   
            "qpos":   right_hand_action,       
            "qvel":   [],       
            "torque": [], 
        }, 
        "body": {
            "qpos": current_body_action,
        }, 
    }
    return colors, depths, states, actions
