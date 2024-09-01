(define (domain task31numericdomain)

  (:requirements :strips :typing :durative-actions :numeric-fluents)

    (:types
        location agent contentType carrier - object
        workstation box content agent - locatable   ;locatable objects
        workstation box  - container            ;objects that can contains something 
        content box - containable                   ;objects that can be contained
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
        (carrier-has-box ?carrier - carrier ?box - box)
        ;occupational logic
        (free ?agent - agent)
    )

    (:functions
        (curr-carrier-load ?carrier - carrier);current load of the carrier: number of boxes in it
        (capacity ?carrier - carrier )               ;capacity of the carrier
    )

    ;Important: temporal planner doesn't support disjunctive precondition so in this domain a box can be emptied in a workstation only if is in it.

    (:durative-action fill-box-from-location
        :parameters (?agent - agent ?box - box ?content - content ?location - location)
        :duration (= ?duration 3)
        :condition (and 
            (at start (free ?agent))
            (at start (is-empty ?box))
            (at start (at ?content ?location))
            (over all (at ?agent ?location))
            (over all (at ?box ?location))
        )
        :effect (and 
            (at start (not (free ?agent)))
            (at start (not (is-empty ?box)))
            (at start (not (at ?content ?location)))
            (at end (contain ?box ?content))
            (at end (free ?agent))
        )
    )

    (:durative-action fill-box-from-workstation
        :parameters (?agent - agent ?box - box ?content - content ?workstation - workstation ?contentType - contentType ?location - location)
        :duration (= ?duration 3)
        :condition (and 
            (at start (free ?agent))
            (at start (is-empty ?box))
            (at start (contain ?workstation ?content))
            (at start (workstation-has-type ?workstation ?contentType))
            (over all (at ?agent ?location))
            (over all (at ?workstation ?location))
            (over all (contain ?workstation ?box))
            (over all (is-type ?content ?contentType))
        )
        :effect (and 
            (at start (not (free ?agent)))
            (at start (not (is-empty ?box)))
            (at start (not (contain ?workstation ?content)))
            (at end (not (workstation-has-type ?workstation ?contentType)))
            (at end (contain ?box ?content))
            (at end (free ?agent))
        )
    )

    (:durative-action empty-box-location
        :parameters (?agent - agent ?box - box ?content - content ?location - location)
        :duration (= ?duration 3)
        :condition (and 
            (at start (free ?agent))
            (at start (contain ?box ?content))
            (over all (at ?agent ?location))
            (over all (at ?box ?location))
        )
        :effect (and 
            (at start (not(free ?agent)))
            (at start (not (contain ?box ?content)))
            (at end (at ?content ?location))
            (at end (is-empty ?box))
            (at end (free ?agent))
        )
    )    

    (:durative-action empty-box-workstation
        :parameters (?agent - agent ?box - box ?content - content ?contentType - contentType ?workstation - workstation ?location - location)
        :duration (= ?duration 3)
        :condition (and 
            (at start (free ?agent))
            (at start (contain ?box ?content))
            (over all (at ?agent ?location))
            (over all (is-type ?content ?contentType))
            (over all (at ?workstation ?location))
            (over all (contain ?workstation ?box))
        )
        :effect (and 
            (at start (not (free ?agent)))
            (at start (not (contain ?box ?content)))
            (at end (workstation-has-type ?workstation ?contentType))
            (at end (is-empty ?box))
            (at end (contain ?workstation ?content))
            (at end (free ?agent))
        )
    )

    (:durative-action pick-up-from-location
        :parameters (?agent - agent ?box - box ?location - location ?carrier - carrier)
        :duration (= ?duration 2)
        :condition (and 
            (at start (free ?agent))
            (over all (< (curr-carrier-load ?carrier)(capacity ?carrier)))
            (at start (at ?box ?location))
            (over all (at ?agent ?location))
            (over all (agent-has-carrier ?agent ?carrier))

        )
        :effect (and 
            (at start (not (free ?agent)))
            (at start (not (at ?box ?location)))
            (at end (increase (curr-carrier-load ?carrier) 1))
            (at end (carrier-has-box ?carrier ?box))
            (at end (free ?agent))
        )
    )

    (:durative-action pick-up-from-workstation
        :parameters (?agent - agent ?workstation - workstation ?location - location ?box - box ?carrier - carrier)
        :duration (= ?duration 2)
        :condition (and 
            (at start (free ?agent))
            (at start (contain ?workstation ?box))
            (over all (< (curr-carrier-load ?carrier)(capacity ?carrier)))
            (over all (at ?workstation ?location))
            (over all (at ?agent ?location))
            (over all (agent-has-carrier ?agent ?carrier))
        )
        :effect (and
            (at start (not (free ?agent)))
            (at start (not (contain ?workstation ?box)))
            (at end (increase (curr-carrier-load ?carrier) 1))
            (at end (carrier-has-box ?carrier ?box))
            (at end (free ?agent))
        )
    )
    
    (:durative-action move
        :parameters (?agent - agent ?from - location ?to - location)
        :duration (= ?duration 5)
        :condition (and 
            (at start (free ?agent))
            (at start (at ?agent ?from))
            (over all (connected ?from ?to))
        )
        :effect (and
            (at start (not (free ?agent)))
            (at start (not (at ?agent ?from)))
            (at end (at ?agent ?to))
            (at end (free ?agent))
        )
    )

    (:durative-action deliver-to-location
        :parameters (?agent - agent ?location - location ?box - box ?carrier - carrier)
        :duration (= ?duration 2)
        :condition (and 
            (at start (free ?agent))
            (over all (carrier-has-box ?carrier ?box))
            (over all (at ?agent ?location))
            (over all (agent-has-carrier ?agent ?carrier))
        )
        :effect (and
            (at start (not (free ?agent)))
            (at end (decrease (curr-carrier-load ?carrier) 1))
            (at end (not (carrier-has-box ?carrier ?box)))
            (at end (at ?box ?location))
            (at end (free ?agent))
        )
    )

    (:durative-action deliver-to-workstation
        :parameters (?agent - agent ?workstation - workstation ?location - location ?box - box ?carrier - carrier)
        :duration (= ?duration 2)
        :condition (and 
            (at start (free ?agent))
            (over all(carrier-has-box ?carrier ?box))
            (over all (agent-has-carrier ?agent ?carrier))
            (over all (at ?agent ?location))
            (over all (at ?workstation ?location))
        )
        :effect (and 
            (at start (not (free ?agent)))
            (at end (not (carrier-has-box ?carrier ?box)))
            (at end (decrease (curr-carrier-load ?carrier) 1))
            (at end (contain ?workstation ?box))
            (at end (free ?agent))
        )
    )

)
