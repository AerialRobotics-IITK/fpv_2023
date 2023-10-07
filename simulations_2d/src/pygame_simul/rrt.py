import pygame
from RRTbasePy import RRTGraph
from RRTbasePy import RRTMap
import time

def main():
    dimensions =(1024,1024)
    start=(50,50)
    goal=(900,900)
    obsdim=30
    obsnum=60
    iteration=0
    t1=0

    pygame.init()
    map=RRTMap(start,goal,dimensions,obsdim,obsnum)
    graph=RRTGraph(start,goal,dimensions,obsdim,obsnum)

    obstacles=graph.makeobs()
    map.drawMap(obstacles)

    t1=time.time()
    while (not graph.path_to_goal()):
        time.sleep(0.1)
        elapsed=time.time()-t1
        t1=time.time()
        #raise exception if timeout
        if elapsed > 10:
            print('timeout re-initiating the calculations')
            raise

        if iteration % 10 == 0:
            X, Y, Parent = graph.bias(goal)
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad*2, 0)
            pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                             map.edgeThickness)

        else:
            X, Y, Parent = graph.expand()
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad*2, 0)
            pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                             map.edgeThickness)

        if iteration % 5 == 0:
            pygame.display.update()
        iteration += 1    
    map.drawPath(graph.getPathCoords())  
    pygame.display.update()
    time.sleep(5)
    pygame.event.clear()
    pygame.event.wait(0)




if __name__ == '__main__':
    result=False
    while not result:
        try:
            main()
            result=True
        except:
            result=False


























