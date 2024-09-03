(define (domain task31numericdomain)

  (:requirements :strips :typing :durative-actions :fluents)

    (:types
        location agent contenttype carrier - object
        workstation box content agent - locatable   ;locatable objects
        workstation box  - container            ;objects that can contains something 
        content box - containable                   ;objects that can be contained
    )

    (:predicates
        (at ?locatable - locatable ?location - location )
        (contain ?container - container ?containable - containable)
        (connected ?loc1 - location ?loc2 - location)
        (workstation_has_type ?workstation - workstation ?contenttype - contenttype) 
        (is_type ?content - content ?contenttype - contenttype)
        (is_empty ?container - container)
        ;Instance no. 2
        (agent_has_carrier ?agent - agent ?carrier - carrier) 
        (carrier_has_box ?carrier - carrier ?box - box)
        ;occupational logic
        (free ?agent - agent)
    )

    (:functions
        (curr_carrier_load ?carrier - carrier);current load of the carrier: number of boxes in it
        (capacity ?carrier - carrier )               ;capacity of the carrier
    )

    ;Important: temporal planner doesn't support disjunctive precondition so in this domain a box can be emptied in a workstation only if is in it.

    (:durative-action fill_box_from_location
        :parameters (?agent - agent ?box - box ?content - content ?location - location)
        :duration (= ?duration 3)
        :condition (and 
            (at start (free ?agent))
            (at start (is_empty ?box))
            (at start (at ?content ?location))
            (over all (at ?agent ?location))
            (over all (at ?box ?location))
        )
        :effect (and 
            (at start (not (free ?agent)))
            (at start (not (is_empty ?box)))
            (at start (not (at ?content ?location)))
            (at end (contain ?box ?content))
            (at end (free ?agent))
        )
    )

    (:durative-action fill_box_from_workstation
        :parameters (?agent - agent ?box - box ?content - content ?workstation - workstation ?contenttype - contenttype ?location - location)
        :duration (= ?duration 3)
        :condition (and 
            (at start (free ?agent))
            (at start (is_empty ?box))
            (at start (contain ?workstation ?content))
            (at start (workstation_has_type ?workstation ?contenttype))
            (over all (at ?agent ?location))
            (over all (at ?workstation ?location))
            (over all (contain ?workstation ?box))
            (over all (is_type ?content ?contenttype))
        )
        :effect (and 
            (at start (not (free ?agent)))
            (at start (not (is_empty ?box)))
            (at start (not (contain ?workstation ?content)))
            (at end (not (workstation_has_type ?workstation ?contenttype)))
            (at end (contain ?box ?content))
            (at end (free ?agent))
        )
    )

    (:durative-action empty_box_location
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
            (at end (is_empty ?box))
            (at end (free ?agent))
        )
    )    

    (:durative-action empty_box_workstation
        :parameters (?agent - agent ?box - box ?content - content ?contenttype - contenttype ?workstation - workstation ?location - location)
        :duration (= ?duration 3)
        :condition (and 
            (at start (free ?agent))
            (at start (contain ?box ?content))
            (over all (at ?agent ?location))
            (over all (is_type ?content ?contenttype))
            (over all (at ?workstation ?location))
            (over all (contain ?workstation ?box))
        )
        :effect (and 
            (at start (not (free ?agent)))
            (at start (not (contain ?box ?content)))
            (at end (workstation_has_type ?workstation ?contenttype))
            (at end (is_empty ?box))
            (at end (contain ?workstation ?content))
            (at end (free ?agent))
        )
    )

    (:durative-action pick_up_from_location
        :parameters (?agent - agent ?box - box ?location - location ?carrier - carrier)
        :duration (= ?duration 2)
        :condition (and 
            (at start (free ?agent))
            (over all (< (curr_carrier_load ?carrier)(capacity ?carrier)))
            (at start (at ?box ?location))
            (over all (at ?agent ?location))
            (over all (agent_has_carrier ?agent ?carrier))

        )
        :effect (and 
            (at start (not (free ?agent)))
            (at start (not (at ?box ?location)))
            (at end(increase (curr_carrier_load ?carrier) 1))
            (at end (carrier_has_box ?carrier ?box))
            (at end (free ?agent))
        )
    )

    (:durative-action pick_up_from_workstation
        :parameters (?agent - agent ?workstation - workstation ?location - location ?box - box ?carrier - carrier)
        :duration (= ?duration 2)
        :condition (and 
            (at start (free ?agent))
            (at start (contain ?workstation ?box))
            (over all (< (curr_carrier_load ?carrier)(capacity ?carrier)))
            (over all (at ?workstation ?location))
            (over all (at ?agent ?location))
            (over all (agent_has_carrier ?agent ?carrier))
        )
        :effect (and
            (at start (not (free ?agent)))
            (at start (not (contain ?workstation ?box)))
            (at end(increase (curr_carrier_load ?carrier) 1))
            (at end (carrier_has_box ?carrier ?box))
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

    (:durative-action deliver_to_location
        :parameters (?agent - agent ?location - location ?box - box ?carrier - carrier)
        :duration (= ?duration 2)
        :condition (and 
            (at start (free ?agent))
            (over all (carrier_has_box ?carrier ?box))
            (over all (at ?agent ?location))
            (over all (agent_has_carrier ?agent ?carrier))
        )
        :effect (and
            (at start (not (free ?agent)))
            (at end (decrease (curr_carrier_load ?carrier) 1))
            (at end (not (carrier_has_box ?carrier ?box)))
            (at end (at ?box ?location))
            (at end (free ?agent))
        )
    )

    (:durative-action deliver_to_workstation
        :parameters (?agent - agent ?workstation - workstation ?location - location ?box - box ?carrier - carrier)
        :duration (= ?duration 2)
        :condition (and 
            (at start (free ?agent))
            (over all(carrier_has_box ?carrier ?box))
            (over all (agent_has_carrier ?agent ?carrier))
            (over all (at ?agent ?location))
            (over all (at ?workstation ?location))
        )
        :effect (and 
            (at start (not (free ?agent)))
            (at end (not (carrier_has_box ?carrier ?box)))
            (at end (decrease (curr_carrier_load ?carrier) 1))
            (at end (contain ?workstation ?box))
            (at end (free ?agent))
        )
    )

)
