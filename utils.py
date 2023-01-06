import hashlib
import numpy as np
import quaternion
import json

def get_token(key,data):
    obj = hashlib.md5(key.encode('utf-8'))
    obj.update(data.encode('utf-8'))
    result = obj.hexdigest()
    return result

def dump(data,path):
    with open(path, "w") as filedata:
        json.dump(data, filedata)

def load(path):
    with open(path, "r") as filedata:
        return json.load(filedata)

def get_intrinsic(fov, image_size_x,image_size_y):
    focal = image_size_x / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = image_size_x / 2.0
    K[1, 2] = image_size_y / 2.0
    return K

def get_rt(transform):
        translation = [transform.location.x,
                        transform.location.y,
                        transform.location.z]
        euler_angles = quaternion.as_euler_angles([transform.rotation.yaw,
                                                    transform.rotation.pitch,
                                                    transform.rotation.roll
                                                    ])
        quat = quaternion.from_euler_angels(euler_angles)
        rotation = quaternion.as_float_array(quat)
        return rotation,translation