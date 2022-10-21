(define (domain kitchen-robot)
(:requirements :typing :equality :fluents)

  (:types         
    location locatable - object
    stovetop countertop drawer - location
    gripper sugarbox spambox - locatable
	robot - gripper
  )

  (:predicates
    (gripper-empty)
    (gripper-holding ?object - locatable)
    (on ?location - location ?object - locatable)     
    (drawer-open)
    (path ?to - location ?from - location)
  )
  
  (:action pickupTop
    :parameters (?food - locatable ?location - location ?gripper - robot)
    :precondition (and 
    (gripper-empty)
    (on ?location ?gripper)
    (on ?location ?food))
    :effect (and 
    not(gripper-empty)
    (gripper-holding ?food))
    )
  
  (:action pickupDrawer
      :parameters (?food - locatable ?drawer - location ?gripper - robot)
      :precondition (and 
      (drawer-open)
      (gripper-empty)
      (on ?drawer ?gripper)
      (on ?drawer ?food))      
      )
      :effect (and 
      not(gripper-empty)
      (gripper-holding ?food))
  
  (:action dropTop
    :parameters (?food - locatable ?location - location ?gripper - robot)
    :precondition (and 
    (gripper-holding ?food)
    (on ?location ?gripper)
    )
    :effect (and 
    (gripper-empty)
    not(gripper-holding ?food)
    (on ?location ?food)))

  (:action dropDrawer
    :parameters (?food - locatable ?drawer - location ?gripper - robot)
    :precondition (and 
    (gripper-holding ?food)
    (on ?drawer ?gripper)
    (drawer-open)
    )
    :effect (and 
    (gripper-empty)
    not(gripper-holding ?food)
    (on ?drawer ?food)))

  (:action openDrawer
      :parameters (?drawer - location ?gripper - robot)
      :precondition (and 
      not(drawer-open)
      (on ?drawer ?gripper)
      )
      :effect (and 
      (drawer-open))
  )

  (:action closeDrawer
      :parameters (?drawer - location ?gripper - robot)
      :precondition (and 
      (drawer-open)
      (on ?drawer ?gripper)
      )
      :effect (and 
      not(drawer-open))
  )

  (:action move
      :parameters (?gripper - robot ?to - location ?from - location)
      :precondition (and 
      (on ?from ?gripper)
      (path ?to ?from)
      )
      :effect (and 
      not(on ?from ?gripper)
      (on ?to ?gripper))
  ) 
)
