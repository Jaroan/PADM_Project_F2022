(define (problem sugarmove-spammove)
	(:domain kitchen-robot)
	(:objects
    	gripper - robot
    	sugarbox - locatable
        spambox - locatable
    	stovetop - location
    	countertop - location
	)
(:init
	(on sugarbox stovetop)
	(gripper-empty)
	(path stovetop countertop)
	(on spambox countertop)
    (not(drawer-open))
    (path countertop drawer)
)


(:goal 
    (and
	    (on sugarbox countertop)
        (on spambox drawer)
        (not (drawer-open))
    )
)
)