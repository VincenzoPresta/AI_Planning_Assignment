(define (domain task1domain)

  (:requirements :strips :typing :universal-preconditions :disjunctive-preconditions)

  (:types
    workstation location box content robot content_type
    
    workstation box content agent - locable
    box workstation - container
    content box - containable
    robot - agent
    volt current - content_type ; Definizione dei tipi di contenuto
  )

(:predicates
    (at ?objectLoc - locable ?location - location )
    (contain ?container - container ?containable - containable)
    (box-is-empty ?box - box)
    (free ?agent - agent)
    (loaded ?agent - agent ?box - box)
    (connected ?loc1 - location ?loc2 - location)
    ;(content-type ?content - content ?type - content_type) 
    (workstation-has-type ?workstation - workstation ?type - content_type) ; Indica di quale tipo di contenuto ha bisogno una workstation
    (is-type ?content - content ?content_type - content_type); Associa contenuti specifici ai loro tipi
)

; ASSUNZIONE: Un contenuto se è nella workstation non è nella location

(:action fill_box_from_location
    :parameters (?agent - agent ?box - box ?content - content ?loc - location )
    :precondition (and 
                        (at ?agent ?loc)
                        (at ?box ?loc)
                        (at ?content ?loc)

                        (free ?agent)
                        (box-is-empty ?box)
    )   
    :effect (and 
                    (not (box-is-empty ?box))
                    (not (at ?content ?loc))
                    (contain ?box ?content)
    )
)

(:action fill_box_from_workstation
    :parameters (?agent - agent ?box - box ?content - content ?workstation - workstation ?type - content_type ?loc - location)
    :precondition (and 
                        (at ?agent ?loc)
                        (at ?workstation ?loc)
                        (contain ?workstation ?box)
                        (contain ?workstation ?content)
                        (is-type ?content ?type)

                        (free ?agent)
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
    :parameters (?agent - agent ?box - box ?content - content ?content_type - content_type ?workstation - workstation ?loc - location)
    :precondition (and 
                        (at ?agent ?loc)
                        (or (at ?box ?loc) (contain ?workstation ?box))
                        (at ?workstation ?loc)

                        (free ?agent)
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
    
                        (free ?agent)
                        (contain ?box ?content)
    )
    :effect (and 
                (not (contain ?box ?content))
                (box-is-empty ?box)

                (at ?content ?loc)
    )
)


(:action pick-up-from-ws
    :parameters (?agent - agent ?box - box ?workstation - workstation ?loc - location)
    :precondition (and 
                        (at ?agent ?loc)
                        (at ?workstation ?loc)
                        (contain ?workstation ?box)
                        
                        (free ?agent)
    )
    :effect (and 
                (not (free ?agent))
                (not (contain ?workstation ?box))
                (loaded ?agent ?box)
    )
)

(:action pick-up-from-location
    :parameters (?agent - agent ?box - box ?loc - location)
    :precondition (and 
                        (at ?agent ?loc)
                        (at ?box ?loc)

                        (free ?agent)
    )
    :effect (and 
                (not (free ?agent))
                (not (at ?box ?loc))
                (loaded ?agent ?box)
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
    :parameters (?agent - agent ?box - box ?workstation - workstation ?loc - location)
    :precondition (and 
                        (at ?agent ?loc)
                        (at ?workstation ?loc)
                        (loaded ?agent ?box)
    )
    :effect (and 
                (not (loaded ?agent ?box))
                (free ?agent)
                (contain ?workstation ?box)
    )
)

(:action deliver-to-loc
    :parameters (?agent - agent ?box - box ?loc - location)
    :precondition (and 
                        (at ?agent ?loc)
                        (loaded ?agent ?box)
    )
    :effect (and 
                (not (loaded ?agent ?box))
                (free ?agent)
                (at ?box ?loc)
    )
)
)
