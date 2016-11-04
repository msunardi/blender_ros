import bpy, math, mathutils
import redis

r = redis.StrictRedis(host="localhost", port=6379, db=0)

Armature = bpy.data.objects["Female1_D3_ConversationGestures"]
Arm = Armature.data

Neck = Arm.bones["Neck"]
PoseNeck = Armature.pose.bones["Neck"]
PoseHead = Armature.pose.bones["Head"]

def fhandler(scene):
    i = bpy.context.scene.frame_current
    bpy.ops.object.mode_set(mode='POSE')
    Armature = bpy.data.objects["Female1_D3_ConversationGestures"]
    r.publish('test2', 'frame: %s' % i)
        
    euler_pose = PoseNeck.rotation_euler
    r.publish('test', "Neck: z:%s, x:%s, y:%s" % (euler_pose[0], euler_pose[1], euler_pose[2]))
    euler_pose = PoseHead.rotation_euler
    r.publish('test', "Head: z:%s, x:%s, y:%s" % (euler_pose[0], euler_pose[1], euler_pose[2]))
    
bpy.app.handlers.frame_change_pre.append(fhandler)
print("Set handler")
r.publish('test3', 'Starting stream to Redis ...')