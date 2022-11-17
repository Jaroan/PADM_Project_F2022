(define (problem sugarmove-spammove)
	(:domain kitchen-robot)
	(:objects
        ;locatables
        arm - robot
    	spam - food
        sugar - food

        ;locations
        counter - top
        stove - top
        drawer - drawer
	)
    (:init
	(on arm counter)
	(on spam counter)
    (on sugar stove)
	(gripper-empty)
    (drawer-closed)
)

    (:goal 
    (and
        (on spam drawer)
        (on sugar counter)
        (drawer-closed)
    )
)

)