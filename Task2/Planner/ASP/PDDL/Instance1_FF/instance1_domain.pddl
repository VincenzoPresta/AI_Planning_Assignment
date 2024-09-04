(define (domain task21domain)

  (:requirements :strips :typing)

  (:types
    location agent contentType - object
    workstation box content agent - locatable
    workstation box - container
    content box - containable
  )
    (:predicates
        (at ?locatable - locatable ?location - location )
        (contain ?container - container ?containable - containable)
        (connected ?loc1 - location ?loc2 - location)
        (workstation-has-type ?workstation - workstation ?contentType - contentType) 
        (is-type ?content - content ?contentType - contentType)
        (is-empty ?box - box)
        (free ?agent)
        (loaded ?agent - agent ?box - box)
    )

    (:action fill-box-from-location
        :parameters (?agent - agent ?box - box ?content - content ?location - location )
        :precondition (and 
                        (at ?agent ?location)
                        (at ?box ?location)
                        (at ?content ?location)
                        (is-empty ?box)
                        (free ?agent)
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
                        (free ?agent)
        )                   
        :effect (and 
                (not (is-empty ?box))
                (not (contain ?workstation ?content))
                (not (workstation-has-type ?workstation ?contentType))
                (contain ?box ?content)
        )
    )

    ;box can be emptied if it is in location or in workstation 
    (:action empty-box-workstation
        :parameters (?agent - agent ?box - box ?content - content ?contentType - contentType ?workstation - workstation ?location - location)
        :precondition (and 
                            (at ?agent ?location)
                            (contain ?workstation ?box)
                            (at ?workstation ?location)
                            (contain ?box ?content)
                            (is-type ?content ?contentType)
                            (free ?agent)
        )
        :effect (and 
                    (not (contain ?box ?content))
                    (is-empty ?box)
                    (contain ?workstation ?content)
                    (workstation-has-type ?workstation ?contentType)
        )
    )

    (:action empty-box-location
        :parameters (?agent - agent ?box - box ?content - content ?location - location)
        :precondition (and 
                            (at ?agent ?location)
                            (at ?box ?location) 
                            (contain ?box ?content)
                            (free ?agent)
        )
        :effect (and 
                    (not (contain ?box ?content))
                    (is-empty ?box)
                    (at ?content ?location)
        )
    )

    (:action pick-up-from-workstation
        :parameters (?agent - agent ?workstation - workstation ?location - location ?box - box)
        :precondition (and 
                    (at ?agent ?location)
                    (at ?workstation ?location)
                    (contain ?workstation ?box)
                    (free ?agent)
        )
        :effect (and 
            (not (contain ?workstation ?box))
            (not (free ?agent))
            (loaded ?agent ?box)
        )
    )

    (:action pick-up-from-location
        :parameters (?agent - agent ?box - box ?location - location)
        :precondition (and 
                            (at ?agent ?location)
                            (at ?box ?location)
                            (free ?agent)
        )
        :effect (and 
                    (not (at ?box ?location))
                    (not (free ?agent))
                    (loaded ?agent ?box)
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

    (:action deliver-to-workstation
        :parameters (?agent - agent ?workstation - workstation ?location - location ?box - box)
        :precondition (and 
                            (at ?agent ?location)
                            (at ?workstation ?location)
                            (loaded ?agent ?box)
        )
        :effect (and 
                    (not (loaded ?agent ?box))
                    (contain ?workstation ?box)
                    (free ?agent)
        )
    )

    (:action deliver-to-location
        :parameters (?agent - agent ?location - location ?box - box)
        :precondition (and 
                            (at ?agent ?location)
                            (loaded ?agent ?box)
        )
        :effect (and 
                    (not (loaded ?agent ?box))
                    (at ?box ?location)
                    (free ?agent)
        )
    )

)

