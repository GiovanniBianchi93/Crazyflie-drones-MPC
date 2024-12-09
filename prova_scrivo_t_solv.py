
file_t_solv = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                  '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                  '/files_trj/t_solv.csv'

t_elapsed = 1.1

with open(file_t_solv, 'a') as file:
        file.write(str(t_elapsed) + '\n')
