(define (domain task1domain)

    (:requirements :strips :typing :disjunctive-preconditions)

    (:types
        location 
        workstation box content agent - locatable   ;locatable objects
        workstation box  - container                ;objects that can contains something 
        content box - containable                   ;objects that can be contained
        robot - agent                               ;useful for having different type of agent
        valve bolt tool - contentType               ;useful for remark in modelling
    )

    (:predicates
        (at ?locatable - locatable ?location - location )
        (contain ?container - container ?containable - containable)
        (is-empty ?container - container)
        (connected ?loc1 - location ?loc2 - location)
        (workstation-has-type ?workstation - workstation ?contentType - contentType) 
        (is-type ?content - content ?contentType - contentType)
        (loaded ?agent - agent ?box - box)
        (free ?agent)
    )
    ; Assumptions:
    ; 1 : A loaded robot CAN'T fill or empty a box, it can move or delivery.


    (:action fill-box-from-location
        :parameters (?agent - agent ?box - box ?content - content ?location - location )
        :precondition (and
                            (free ?agent)
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
        :parameters (?agent - agent ?box - box ?content - content ?workstation - workstation ?contentType - contentType ?loc - location)
        :precondition (and
                            (free ?agent)
                            (at ?agent ?loc)
                            (at ?workstation ?loc)
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

    ;box can be emptied if it is in location or in workstation
    (:action empty-box-workstation
        :parameters (?agent - agent ?box - box ?content - content ?contentType - contentType ?workstation - workstation ?location - location)
        :precondition (and 
                            (free ?agent)
                            (at ?agent ?location)
                            (or (at ?box ?location) (contain ?workstation ?box))
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

    (:action empty-box-location
        :parameters (?agent - agent ?box - box ?content - content ?location - location)
        :precondition (and 
                            (free ?agent)
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


    (:action pick-up-from-workstation
        :parameters (?agent - agent ?box - box ?workstation - workstation ?location - location)
        :precondition (and 
                            (free ?agent)
                            (at ?agent ?location)
                            (at ?workstation ?location)
                            (contain ?workstation ?box)
        )
        :effect (and 
                    (not (free ?agent))
                    (not (contain ?workstation ?box))
                    (loaded ?agent ?box)
        )
    )

    (:action pick-up-from-location
        :parameters (?agent - agent ?box - box ?location - location)
        :precondition (and 
                            (free ?agent)
                            (at ?agent ?location)
                            (at ?box ?location)
        )
        :effect (and 
                    (not (free ?agent))
                    (not (at ?box ?location))
                    (loaded ?agent ?box)
        )
    )

    (:action move
        :parameters (?agent - agent ?from - location ?to - location)
        :precondition (and 
                        (or (connected ?from ?to) (connected ?to ?from))
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
                    (free ?agent)
                    (contain ?workstation ?box)
        )
    )

    (:action deliver-to-location
        :parameters (?agent - agent ?box - box ?location - location)
        :precondition (and 
                            (at ?agent ?location)
                            (loaded ?agent ?box)
        )
        :effect (and 
                    (not (loaded ?agent ?box))
                    (free ?agent)
                    (at ?box ?location)
        )
    )
)
