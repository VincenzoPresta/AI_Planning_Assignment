(define (domain task22domain)

  (:requirements :strips :typing)

  (:types
    location agent contentType carrier - object
    workstation box content agent - locatable
    workstation box slot - container
    content box - containable
  )
    (:predicates
        (at ?locatable - locatable ?location - location )
        (contain ?container - container ?containable - containable)
        (connected ?loc1 - location ?loc2 - location)
        (workstation-has-type ?workstation - workstation ?contentType - contentType) 
        (is-type ?content - content ?contentType - contentType)
        (is-empty ?container - container)
        ;Instance no. 2
        (agent-has-carrier ?agent - agent ?carrier - carrier)             
        (carrier-has-slot ?carrier - carrier ?slot - slot)
    )

    (:action fill-box-from-location
        :parameters (?agent - agent ?box - box ?content - content ?location - location )
        :precondition (and 
                        (at ?agent ?location)
                        (at ?box ?location)
                        (at ?content ?location)
                        (is-empty ?box)
        )      
        :effect (and 
                    (not (is-empty ?box))
                    (not (at ?content ?location))
                    (contain ?box ?content)
        )
    )

    (:action fill-box-from-workstation
        :parameters (?agent - agent ?box - box ?content - content ?workstation - workstation ?contentType - contentType ?location - location)
        :precondition (and 
                        (at ?agent ?location)
                        (at ?workstation ?location)
                        (contain ?workstation ?box)
                        (contain ?workstation ?content)
                        (is-type ?content ?contentType)
                        (is-empty ?box)
        )                   
        :effect (and 
                (not (is-empty ?box))
                (not (contain ?workstation ?content))
                (not (workstation-has-type ?workstation ?contentType))
                (contain ?box ?content)
        )
    )

    (:action empty-box-location
        :parameters (?agent - agent ?box - box ?content - content ?location - location)
        :precondition (and 
                            (at ?agent ?location)
                            (at ?box ?location) 
                            (contain ?box ?content)
        )
        :effect (and 
                    (not (contain ?box ?content))
                    (is-empty ?box)
                    (at ?content ?location)
        )
    )

    ;box can be emptied in a workstation only if it is in the workstation
    (:action empty-box-workstation
        :parameters (?agent - agent ?box - box ?content - content ?contentType - contentType ?workstation - workstation ?location - location)
        :precondition (and 
                            (at ?agent ?location)
                            (contain ?workstation ?box)
                            (at ?workstation ?location)
                            (contain ?box ?content)
                            (is-type ?content ?contentType)
        )
        :effect (and 
                    (not (contain ?box ?content))
                    (is-empty ?box)
                    (contain ?workstation ?content)
                    (workstation-has-type ?workstation ?contentType)
        )
    )

    (:action pick-up-from-location
        :parameters (?agent - agent ?box - box ?location - location ?carrier - carrier ?slot - slot)
        :precondition (and 
                            (at ?agent ?location)
                            (at ?box ?location)
                            (agent-has-carrier ?agent ?carrier)
                            (carrier-has-slot ?carrier ?slot)
                            (is-empty ?slot)
        )
        :effect (and 
                    (not (at ?box ?location))
                    (not (is-empty ?slot))
                    (contain ?slot ?box)
        )
    )

    (:action pick-up-from-workstation
        :parameters (?agent - agent ?workstation - workstation ?location - location ?box - box  ?carrier - carrier ?slot - slot)
        :precondition (and 
                    (at ?agent ?location)
                    (at ?workstation ?location)
                    (contain ?workstation ?box)
                    (agent-has-carrier ?agent ?carrier)
                    (carrier-has-slot ?carrier ?slot)
                    (is-empty ?slot)
        )
        :effect (and 
            (not (contain ?workstation ?box))
            (not (is-empty ?slot))
            (contain ?slot ?box)
        )
    )

    (:action move
        :parameters (?agent - agent ?from - location ?to - location)
        :precondition (and 
                        (connected ?from ?to)
                        (at ?agent ?from)
        )
        :effect (and 
                    (not (at ?agent ?from))
                    (at ?agent ?to)
        )
    )

    (:action deliver-to-location
        :parameters (?agent - agent ?location - location ?box - box ?carrier - carrier ?slot - slot)
        :precondition (and 
                            (at ?agent ?location)
                            (agent-has-carrier ?agent ?carrier)
                            (carrier-has-slot ?carrier ?slot)
                            (contain ?slot ?box)
        )
        :effect (and 
                    (not (contain ?slot ?box))
                    (is-empty ?slot)
                    (at ?box ?location)
        )
    )

    (:action deliver-to-workstation
        :parameters (?agent - agent ?workstation - workstation ?location - location ?box - box ?carrier - carrier ?slot - slot)
        :precondition (and 
                            (at ?agent ?location)
                            (at ?workstation ?location)
                            (agent-has-carrier ?agent ?carrier)
                            (carrier-has-slot ?carrier ?slot)
                            (contain ?slot ?box)
        )
        :effect (and 
                    (not (contain ?slot ?box))
                    (is-empty ?slot)
                    (contain ?workstation ?box)
        )
    )

)

