import os
import math
from collections import deque
## Usable frames
usable_frames = {
    'walk1_subject1': [(100, 1250), (2200, 3685), (6000, 6460)],
    'walk1_subject2': [(500, 1520), (1620, 7750)], # max: 7840
    'walk1_subject5': [(80, 7779)], # max: 7840
    'walk3_subject1': [(70, 1900), (6650, 7338)], # max: 7399
    'walk3_subject2': [(900, 2900), (3650, 7300)],  # max: 7399
    'run1_subject2': [(95, 6600)], # 7135
    'run1_subject5': [(100, 7040)],  # 7135
    'run2_subject1': [(100, 7240)], # 7345
    'sprint1_subject2': [(100, 8000)], # 8194
}

# cur_dir =  os.getcwdb()

outputs_dir = "D:/Unity/Unity 2021 Editor Test/Assets/outputs/" # sprint1_subject2_output.txt
newfile_dir = "D:/Unity/Unity 2021 Editor Test/Python/pyoutputs/"

# My serach vector length:
# 12 + 3 (what they had)
# 5 * 3 (they used 2 numbers for traj and orientation, I use 2 numbers for traj and 3 for orientation - I think I only need 1 for orientation but being extra safe
# = 30
search_vec_len = 30

def clean_data():
    # Final search vector should be:
    # left and right foot local positions and global velocities (2 pairs of 2 vectors in R^3 => 12 numbers)
    # hip global velocity (one number in R^3 => 3 numbers)
    # hip trajectory positions and orientations located at 20, 40, and 60 frames in the future
    # for me: last number that says what index this goes to
    for name, ranges in usable_frames.items():
        finalContents = deque()
        with open(outputs_dir + name + "_output.txt") as f:
            # get rid of opening line
            f.readline()
            contents = f.readlines()
            hipTrajAndOrientation = {}
            for frameRange in reversed(ranges):
                start, end = frameRange[0], frameRange[1]
                for i in range(end + 60, start - 1, -1): # iterate backwards
                    rawData = contents[i]
                    # Idx 757 : [ -1.136636 , 0.07606596 , 5.391357 , -0.9437488 , 0.2256934 , 5.587147 , -0.008511577 , 0.01436976 , 0.00360691 , -1.45846 , -0.06062899 , -0.3187729 , -0.5651425 , -0.01554871 , -0.1286302 , -0.7252386 , 20.93792 , 353.6751 , 264.6157 , ]
                    trimmedString = rawData[rawData.index("[") + 1:-4] # turns "idx 0: [-1,  -2]" to "-1, -2"
                    strValues = [s.strip() for s in trimmedString.split(",")]
                    hipTrajAndOrientation[i] = strValues[15:]
                    if i > end:
                        continue
                    currentHipPosX = float(strValues[15])
                    currentHipPosZ = float(strValues[16])
                    finalVal = strValues[:15] # + hipTrajAndOrientation[i + 20] + hipTrajAndOrientation[i + 40] + hipTrajAndOrientation[i + 60] + [str(i)]
                    for j in [20, 40, 60]:
                        futureHipPosX = float(hipTrajAndOrientation[i + j][0])
                        futureHipPosZ = float(hipTrajAndOrientation[i + j][1])
                        finalVal += [str(futureHipPosX - currentHipPosX) , str(futureHipPosZ - currentHipPosZ)]
                        finalVal += hipTrajAndOrientation[i + j][2:]
                    finalVal += [str(i)]

                    finalContents.appendleft(",".join(finalVal))
            with open(newfile_dir + name + "_unnormalized_outputs.txt", 'w') as outfile:
                outfile.write("\n".join(finalContents))
# compute the mean and std dev for each feature across every frame and then normalize each feature
# independently by subtracting its mean and dividing by its std dev: (v(i) - v_mean) / v_std_dev
means =  [0 for i in range(search_vec_len)] # maps index to mean
std_devs =  [0 for i in range(search_vec_len)]
def get_mean_and_std_dev():

    means_helper =  [[0,0] for i in range(search_vec_len)] # maps index to (current_sum, num_seen)

    for name in usable_frames.keys():
        with open(newfile_dir + name + "_unnormalized_outputs.txt") as f:
            contents = f.readlines()
            for line in contents:
                vals = line.split(',')
                if len(vals) != 31:
                    raise Exception("WTF", len(vals))
                for i in range(search_vec_len): # last val is index
                    means_helper[i][0] += float(vals[i])
                    means_helper[i][1] += 1

    for i in range(search_vec_len):
        means[i] =  means_helper[i][0] / means_helper[i][1]

    std_dev_helper = [[0,0] for i in range(search_vec_len)] # maps index to (sum so far of (x - mean(x)) , num seen)
    for name in usable_frames.keys():
        with open(newfile_dir + name + "_unnormalized_outputs.txt") as f:
            contents = f.readlines()
            for line in contents:
                vals = line.split(',')
                if len(vals) != 31:
                    raise Exception("WTF")
                for i in range(search_vec_len): # last val is index
                    std_dev_helper[i][0] += pow(float(vals[i]) - means[i] , 2)
                    std_dev_helper[i][1] += 1

    for i in range(search_vec_len):
        stats = std_dev_helper[i]
        std_devs[i] = math.sqrt(stats[0] / (stats[1] - 1))

    maxXVel, maxZVel = get_max_hip_vel()
    with open(newfile_dir + "stats.txt", 'w') as f:
        f.write("Means: \n" + ",".join([str(val) for val in means]))
        f.write("\nStd_Devs: \n" + ",".join([str(val) for val in std_devs]))
        f.write("\nMax X vel: " + str(maxXVel) + "  Max Z vel: " + str(maxZVel))


def normalizeData():
    get_mean_and_std_dev()
    for name, ranges in usable_frames.items():
        with open(newfile_dir + name + "_unnormalized_outputs.txt") as infile:
            # get rid of opening line
            contents = infile.readlines()
            with open(newfile_dir + name + "_normalized_outputs.txt", 'w') as outfile:
                for line in contents:
                    values = [ float(val) for val in line.split(',')]
                    for i in range(len(values) - 1):
                        values[i] = (values[i] - means[i]) / std_devs[i]
                    outfile.write(",".join([str(val) for val in values])+ "\n")

def get_max_hip_vel():
    maxXVel = 0
    maxZVel = 0
    for name in usable_frames.keys():
        with open(newfile_dir + name + "_unnormalized_outputs.txt") as f:
            contents = f.readlines()
            for line in contents:
                vals = line.split(',')
                maxXVel = max(maxXVel, float(vals[12]))
                maxZVel = max(maxZVel, float(vals[14]))
    print("Max X vel: " + str(maxXVel))
    print("Max Z vel: " + str(maxZVel))
    return maxXVel, maxZVel


# clean_data()
# normalizeData()
get_max_hip_vel()

# unity search vector code
# return new float[] {
#                 leftFootLocalPos.x, 0
#                 leftFootLocalPos.y, 1
#                 leftFootLocalPos.z, 2
#                 rightFootLocalPos.x, 3
#                 rightFootLocalPos.y, 4
#                 rightFootLocalPos.z, 5
#                 leftFootGlobalVelocity.x, 6
#                 leftFootGlobalVelocity.y, 7
#                 leftFootGlobalVelocity.z, 8
#                 rightFootGlobalVelocity.x, 9
#                 rightFootGlobalVelocity.y, 10
#                 rightFootGlobalVelocity.z, 11
#                 hipGlobalVel.x, 12
#                 hipGlobalVel.y, 13
#                 hipGlobalVel.z, # 14
#                 hip.transform.position.x, 15
#                 hip.transform.position.z, 16
#                 hip.transform.rotation.eulerAngles.x, 17
#                 hip.transform.rotation.eulerAngles.y, 18
#                 hip.transform.rotation.eulerAngles.z, 19
#         };

