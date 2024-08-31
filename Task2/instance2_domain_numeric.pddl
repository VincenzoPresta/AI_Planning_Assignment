(define (domain task22domainwithfluents)

    (:requirements :strips :typing :numeric-fluents)

    (:types
        location agent contentType carrier - object
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
        (is-empty ?container - container)
        ;Instance no. 2 with numeric fluents
        ;No need slots, handling of capacity is done with functions
        (agent-has-carrier ?agent - agent ?carrier - carrier)
        (carrier-has-box ?carrier - carrier ?box - box)
    )

    (:functions
        (curr-carrier-load ?carrier - carrier);current load of the carrier: number of boxes in it
        (capacity ?carrier - carrier )               ;capacity of the carrier
    )

    ;in this version disjunctive-preconditions are not used because the planner used (OPTIC) doesn't support them. 

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

    ;box can be emptied if it is in location or in workstation 
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

    (:action pick-up-from-workstation
        :parameters (?agent - agent ?workstation - workstation ?location - location ?box - box  ?carrier - carrier)
        :precondition (and 
                    (at ?agent ?location)
                    (at ?workstation ?location)
                    (contain ?workstation ?box)
                    (agent-has-carrier ?agent ?carrier)
                    (< (curr-carrier-load ?carrier) (capacity ?carrier))
        )
        :effect (and 
            (not (contain ?workstation ?box))
            (carrier-has-box ?carrier ?box)
            (increase (curr-carrier-load ?carrier) 1)
        )
    )

    (:action pick-up-from-location
        :parameters (?agent - agent ?box - box ?location - location ?carrier - carrier)
        :precondition (and 
                            (at ?agent ?location)
                            (at ?box ?location)
                            (agent-has-carrier ?agent ?carrier)
                            (< (curr-carrier-load ?carrier) (capacity ?carrier))
        )
        :effect (and 
                    (not (at ?box ?location))
                    (carrier-has-box ?carrier ?box)
                    (increase (curr-carrier-load ?carrier) 1)
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
        :parameters (?agent - agent ?workstation - workstation ?location - location ?box - box ?carrier - carrier)
        :precondition (and 
                            (at ?agent ?location)
                            (at ?workstation ?location)
                            (agent-has-carrier ?agent ?carrier)
                            (carrier-has-box ?carrier ?box)
        )
        :effect (and 
                    (not (carrier-has-box ?carrier ?box))
                    (contain ?workstation ?box)
                    (decrease (curr-carrier-load ?carrier) 1)
        )
    )

    (:action deliver-to-location
        :parameters (?agent - agent ?location - location ?box - box ?carrier - carrier)
        :precondition (and 
                            (at ?agent ?location)
                            (agent-has-carrier ?agent ?carrier)
                            (carrier-has-box ?carrier ?box)
        )
        :effect (and 
                    (not (carrier-has-box ?carrier ?box))
                    (at ?box ?location)
                    (decrease (curr-carrier-load ?carrier) 1)
        )
    )

)

