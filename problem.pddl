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
    (on countertop gripper)
	(on sugarbox stovetop)
	(gripper-empty)
	(on spambox countertop)
    (not(drawer-open))
)


(:goal 
    (and
	    (on sugarbox countertop)
        (on spambox drawer)
        (not (drawer-open))
    )
)
)