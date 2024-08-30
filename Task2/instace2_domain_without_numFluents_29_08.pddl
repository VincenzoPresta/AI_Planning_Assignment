(define (domain task22domain)

  (:requirements :strips :typing :disjunctive-preconditions)

  (:types
    workstation location box content robot contentType 

    carrier slot
    
    workstation box content agent - locable
    box workstation - container
    content box - containable
    robot - agent
    volt current - contentType ; Definizione dei tipi di contenuto
    cart - carrier              ; carrier is generic, there can be more carrier type
  )

    (:predicates
        (at ?objectLoc - locable ?location - location )
        (contain ?container - container ?containable - containable)
        (box-is-empty ?box - box)
        (connected ?loc1 - location ?loc2 - location)
        (workstation-has-type ?workstation - workstation ?type - contentType) ; Indica di quale tipo di contenuto ha bisogno una workstation
        (is-type ?content - content ?content_type - contentType); Associa contenuti specifici ai loro tipi

        ;modifiche per istanza no. 2
        (agent-has-carrier ?agent - agent ?carrier - carrier)             ;true se agent ha il carrier
        (carrier-has-slot ?carrier - carrier ?slot - slot)
        (slot-has-box ?slot - slot ?box - box)
        (free ?slot - slot)
    )

    (:action fill-box-from-location
        :parameters (?agent - agent ?box - box ?content - content ?loc - location )
        :precondition (and 
                        (at ?agent ?loc)
                        (at ?box ?loc)
                        (at ?content ?loc)

                        ;(free ?agent)
                        (box-is-empty ?box)
        )      
        :effect (and 
                    (not (box-is-empty ?box))
                    (not (at ?content ?loc))
                    (contain ?box ?content)
        )
    )

    (:action fill-box-from-workstation
        :parameters (?agent - agent ?box - box ?content - content ?workstation - workstation ?type - contentType ?loc - location)
        :precondition (and 
                        (at ?agent ?loc)
                        (at ?workstation ?loc)
                        (contain ?workstation ?box)
                        (contain ?workstation ?content)
                        (is-type ?content ?type)

                        ;(free ?agent)
                        (box-is-empty ?box)
        )                   
        :effect (and 
                (not (box-is-empty ?box))
                (not (contain ?workstation ?content))
                (not (workstation-has-type ?workstation ?type))
                (contain ?box ?content)
        )
    )

    ; La box può essere svuotata sia se è nella workstation sia se è nella location
    (:action empty-box-ws
        :parameters (?agent - agent ?box - box ?content - content ?content_type - contentType ?workstation - workstation ?loc - location)
        :precondition (and 
                            (at ?agent ?loc)
                            (or (at ?box ?loc) (contain ?workstation ?box))
                            (at ?workstation ?loc)

                            ;(free ?agent)
                            (contain ?box ?content)
                            (is-type ?content ?content_type)
        )
        :effect (and 
                    (not (contain ?box ?content))
                    (box-is-empty ?box)
                    (contain ?workstation ?content)
                    (workstation-has-type ?workstation ?content_type)
        )
    )

    (:action empty-box-loc
        :parameters (?agent - agent ?box - box ?content - content ?loc - location)
        :precondition (and 
                            (at ?agent ?loc)
                            (at ?box ?loc) 
        
                            ;(free ?agent)
                            (contain ?box ?content)
        )
        :effect (and 
                    (not (contain ?box ?content))
                    (box-is-empty ?box)

                    (at ?content ?loc)
        )
    )

    (:action pick-up-from-ws
        :parameters (?agent - agent ?carrier - carrier ?slot - slot ?box - box ?workstation - workstation ?loc - location)
        :precondition (and 
                    (at ?agent ?loc)
                    (at ?workstation ?loc)
                    (contain ?workstation ?box)
                    (agent-has-carrier ?agent ?carrier)
                    (carrier-has-slot ?carrier ?slot)
                    (free ?slot)
        )
        :effect (and 
            (not (contain ?workstation ?box))
            (not (free ?slot))
            (slot-has-box ?slot ?box)
        )
    )

    (:action pick-up-from-location
        :parameters (?agent - agent ?carrier - carrier ?slot - slot ?box - box ?loc - location)
        :precondition (and 
                            (at ?agent ?loc)
                            (at ?box ?loc)
                            (agent-has-carrier ?agent ?carrier)
                            (carrier-has-slot ?carrier ?slot)
                            (free ?slot)

        )
        :effect (and 
                    (not (at ?box ?loc))
                    (not (free ?slot))
                    (slot-has-box ?slot ?box)
        )
    )

    (:action move
        :parameters (?agent - agent ?loc1 - location ?loc2 - location)
        :precondition (and 
                        (or (connected ?loc1 ?loc2) (connected ?loc2 ?loc1))
                        (at ?agent ?loc1)
        )
        :effect (and 
                    (not (at ?agent ?loc1))
                    (at ?agent ?loc2)
        )
    )

    (:action deliver-to-ws
        :parameters (?agent - agent ?carrier - carrier ?slot - slot ?box - box ?workstation - workstation ?loc - location)
        :precondition (and 
                            (at ?agent ?loc)
                            (at ?workstation ?loc)
                            
                            (agent-has-carrier ?agent ?carrier)
                            (carrier-has-slot ?carrier ?slot)
                            (slot-has-box ?slot ?box)
        )
        :effect (and 
                    (not (slot-has-box ?slot ?box))
                    (free ?slot)
                    (contain ?workstation ?box)
        )
    )

    (:action deliver-to-loc
        :parameters (?agent - agent ?carrier - carrier ?slot - slot ?box - box ?loc - location)
        :precondition (and 
                            (at ?agent ?loc)
                            (agent-has-carrier ?agent ?carrier)
                            (carrier-has-slot ?carrier ?slot)
                            (slot-has-box ?slot ?box)
        )
        :effect (and 
                    (not (slot-has-box ?slot ?box))
                    (free ?slot)
                    (at ?box ?loc)
        )
    )

)

