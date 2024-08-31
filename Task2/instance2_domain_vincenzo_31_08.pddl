(define (domain task22domain)

  (:requirements :strips :typing :disjunctive-preconditions)

   (:types
    locatable                                ; General type for objects that can be located
    container                                ; General type for objects that can contain other objects
    containable                              ; General type for objects that can be contained
    contentType                              ; General type for different types of content
    carrier                                  ; General type for objects that can carry other objects
    agent                                    ; General type for robotic agents
    
    location - locatable                     ; Locations are locatable objects
    agent - locatable                        ; Agents are locatable objects
    workstation - locatable container        ; Workstations are both locatable and containers
    box - locatable containable container    ; Boxes are locatable, containable, and containers
    slot - container                         ; Slots are containers
    robot - agent                            ; Robots are a specific type of agent
    cart -  carrier                          ; Carts are carriers
    content - containable                    ; Contents are objects that can be contained
    valve bolt tool - contentType            ; Specific types of content
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

    ;box can be emptied if it is in location or in workstation 
    (:action empty-box-workstation
        :parameters (?agent - agent ?box - box ?content - content ?contentType - contentType ?workstation - workstation ?location - location)
        :precondition (and 
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

)

