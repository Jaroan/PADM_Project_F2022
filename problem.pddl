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
        init - init
	)
    (:init
	(on arm init)
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