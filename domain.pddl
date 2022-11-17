(define (domain kitchen-robot)
(:requirements :typing :negative-preconditions) 

(:types         
    location locatable - object
	bot food - locatable
    robot - bot
    drawer - location
    top - location
)

(:predicates
	(on ?obj - locatable ?loc - location)
	(holding ?arm - locatable ?food - locatable)
    (gripper-empty)
    (drawer-closed)
)

(:action pickupTop
  :parameters
   (?arm - bot
    ?food - locatable
    ?loc - top)
  :precondition
   (and 
      (on ?arm ?loc) 
      (on ?food ?loc) 
      (gripper-empty)
    )
  :effect
   (and 
      (not (on ?food ?loc))
      (holding ?arm ?food)
      (not (gripper-empty))
   )
)

(:action dropTop
  :parameters
   (?arm - bot
    ?food - locatable
    ?loc - top)
  :precondition
   (and 
      (on ?arm ?loc)
      (holding ?arm ?food)
    )
  :effect
   (and 
      (on ?food ?loc)
      (gripper-empty)
      (not (holding ?arm ?food))
   )
)

(:action pickupDrawer
  :parameters
   (?arm - bot
    ?food - locatable
    ?loc - drawer)
  :precondition
   (and 
      (on ?arm ?loc) 
      (on ?food ?loc) 
      (gripper-empty)
      (not(drawer-closed))
    )
  :effect
   (and 
      (not (on ?food ?loc))
      (holding ?arm ?food)
      (not (gripper-empty))
   )
)

(:action dropDrawer
  :parameters
   (?arm - bot
    ?food - locatable
    ?loc - drawer)
  :precondition
   (and 
      (on ?arm ?loc)
      (holding ?arm ?food)
      (not(drawer-closed))
    )
  :effect
   (and 
      (on ?food ?loc)
      (gripper-empty)
      (not (holding ?arm ?food))
   )
)

(:action openDrawer
  :parameters
   (?arm - bot
    ?loc - drawer)
  :precondition
   (and 
      (on ?arm ?loc)
      (gripper-empty)
      (drawer-closed)
    )
  :effect
   (and 
      (not(drawer-closed))
   )
)

(:action closeDrawer
  :parameters
   (?arm - bot
    ?loc - drawer)
  :precondition
   (and 
      (on ?arm ?loc)
      (gripper-empty)
      (not(drawer-closed))
    )
  :effect
   (and 
      (drawer-closed)
   )
)

(:action move
  :parameters
   (?arm - bot
    ?from - location
    ?to - location)
  :precondition
   ( and
    (on ?arm ?from) 
   )
  :effect
   (and 
    (not (on ?arm ?from))
    (on ?arm ?to)
   )
)

)