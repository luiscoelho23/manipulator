import csv
import load_rl

#state h_ang[3] h_ee[2] index

human_ang = []
human_ee = []
index = 0

with open("/home/luisc/ws_manipulator/src/manipulator/resources/dmp/3dmp.csv","r") as csvfile:
    reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)
    for row in reader:
        human_ang.append(row)


with open("/home/luisc/ws_manipulator/src/manipulator/resources/dmp/ee_trajectory.csv","r") as csvfile:
    next(csvfile)
    reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)
    for row in reader:
        human_ee.append(row)

with open('file.txt', mode='w') as file:
    while index < 149 :
        action = load_rl.get_action((human_ang[index][0],human_ang[index][1],human_ang[index][2],human_ee[index][0], human_ee[index][2],index))
        string = (action[0] +human_ang[index][0]).astype(str) + ';' + (action[1] +human_ang[index][1]).astype(str) + ';' + (action[2] + human_ang[index][2]).astype(str) + '\n'
        file.write(string)
        index += 1

