(define (domain task31domain)

  (:requirements :strips :typing :disjunctive-preconditions :durative-actions)

  (:types
    workstation location box content robot content_type 

    carrier slot
    
    workstation box content agent - locable
    box workstation - container
    content box - containable
    robot - agent
    volt current - content_type ; Definizione dei tipi di contenuto
    cart - carrier              ; carrier is generic, there can be more carrier type
  )

    (:predicates
        (at ?objectLoc - locable ?location - location )
        (contain ?container - container ?containable - containable)
        (box-is-empty ?box - box)
        (connected ?loc1 - location ?loc2 - location)
        (workstation-has-type ?workstation - workstation ?type - content_type) ; Indica di quale tipo di contenuto ha bisogno una workstation
        (is-type ?content - content ?content_type - content_type); Associa contenuti specifici ai loro tipi

        ;modifiche per istanza no. 2
        (agent-has-carrier ?agent - agent ?carrier - carrier)             ;true se agent ha il carrier
        (carrier-has-slot ?carrier -carrier ?slot - slot)
        (slot-has-box ?slot - slot ?box - box)
        (free ?slot - slot)
    )


    ;Converti il dominio definito nel paragrafo 2.2 per utilizzare azioni durative. Scegli durate arbitrarie ma ragionevoli per le diverse azioni.
    ;Considera la possibilità di eseguire azioni in parallelo quando ciò sarebbe possibile nella realtà. 
    ;Ad esempio, un agente robotico non può raccogliere diverse scatole contemporaneamente, né raccogliere una scatola e, se è un drone, volare verso una destinazione allo stesso tempo.


    ;check se serve la free del robot
    ;check durate
    
    (:durative-action fill_box_from_location
        :parameters (?agent - agent ?box - box ?content - content ?loc - location)
        :duration (= ?duration 3)
        :condition (and 
            (at start (and 
                (box-is-empty ?box)
                (at ?content ?loc)                
            ))
            (over all (and 
                (at ?agent ?loc)
                (at ?box ?loc)
            ))
        )
        :effect (and 
            (at start (and 
                (not (box-is-empty ?box))
                (not (at ?content ?loc))
            ))
            (at end (and 
                (contain ?box ?content)
            ))
        )
    )

    (:durative-action fill_box_from_workstation
        :parameters (?agent - agent ?box - box ?content - content ?workstation - workstation ?type - content_type ?loc - location)
        :duration (= ?duration 3)
        :condition (and 
            (at start (and 
                (box-is-empty ?box)
                (contain ?workstation ?content)
                (workstation-has-type ?workstation ?type)  
            ))
            (over all (and 
                (at ?agent ?loc)
                (at ?workstation ?loc)
                (contain ?workstation ?box)    
                (is-type ?content ?type)
            ))
        )
        :effect (and 
            (at start (and 
                (not (box-is-empty ?box))
                (not (contain ?workstation ?content))
            ))
            (at end (and 
                (not (workstation-has-type ?workstation ?type))
                (contain ?box ?content)
            ))
        )
    )

    (:durative-action empty-box-workstation
        :parameters (?agent - agent ?box - box ?content - content ?type - content_type ?workstation - workstation ?location - location)
        :duration (= ?duration 3)
        :condition (and 
            (at start (and 
                (contain ?box ?content)
            ))
            (over all (and 
                (at ?agent ?location)
                (is-type ?content ?type)
                (at ?workstation ?location)
                (or (at ?box ?location) (contain ?workstation ?box))
            ))
        )
        :effect (and 
            (at start (and 
                (workstation-has-type ?workstation ?type) ;check
            ))
            (at end (and 
                (not (contain ?box ?content))
                (box-is-empty ?box)
                (contain ?workstation ?content)
            ))
        )
    )

    (:durative-action empty-box-location
        :parameters (?agent - agent ?box - box ?content - content ?location - location)
        :duration (= ?duration 3)
        :condition (and 
            (at start (and 
                (contain ?box ?content)
            ))
            (over all (and 
                (at ?agent ?location)
                (at ?box ?location)
            ))
        )
        :effect (and 
            (at start (and 

            ))
            (at end (and 
                (not (contain ?box ?content))
                (at ?content ?location)
                (box-is-empty ?box)
            ))
        )
    )

    (:durative-action pick-up-from-location
        :parameters (?agent - agent ?carrier - carrier ?slot - slot ?box - box ?location - location )
        :duration (= ?duration 2)
        :condition (and 
            (at start (and 
                (free ?slot)
                (at ?box ?location)
            ))
            (over all (and 

                (at ?agent ?location)
                (agent-has-carrier ?agent ?carrier)
                (carrier-has-slot ?carrier ?slot)

            ))
        )
        :effect (and 
            (at start (and 
            ))
            (at end (and 
                (not (free ?slot))
                (not (at ?box ?location ))
                (slot-has-box ?slot ?box)
            ))
        )
    )

    (:durative-action pick-up-from-workstation
        :parameters (?agent - agent ?carrier - carrier ?slot - slot ?box - box ?workstation - workstation ?location - location)
        :duration (= ?duration 2)
        :condition (and 
            (at start (and 
                (contain ?workstation ?box)
                (free ?slot)
            ))
            (over all (and 
                (at ?agent ?location)
                (at ?workstation ?location)
                (agent-has-carrier ?agent ?carrier)
                (carrier-has-slot ?carrier ?slot)

            ))
        )
        :effect (and 
            (at start (and 


            ))
            (at end (and 
                (not (contain ?workstation ?box))
                (not (free ?slot))
                (slot-has-box ?slot ?box)
            ))
        )
    )
    

    (:durative-action move
        :parameters (?agent - agent ?location1 - location ?location2 - location)
        :duration (= ?duration 5)
        :condition (and 
            (at start (and 
                (at ?agent ?location1)
            ))
            (over all (and 
                (or (connected ?location1 ?location2) (connected ?location2 ?location1))
            ))
        )
        :effect (and 
            (at start (and 
                (not (at ?agent ?location1))

            ))
            (at end (and 
                (at ?agent ?location2)
            ))
        )
    )


    (:durative-action deliver-to-workstation
        :parameters (?agent - agent ?carrier - carrier ?slot - slot ?box - box ?workstation - workstation ?location - location)
        :duration (= ?duration 2)
        :condition (and 
            (at start (and 
                (slot-has-box ?slot ?box)
            ))
            (over all (and 
                (at ?agent ?location)
                (at ?workstation ?location)
                (agent-has-carrier ?agent ?carrier)
                (carrier-has-slot ?carrier ?slot)
            ))
        )
        :effect (and 
            (at start (and 
                (not (slot-has-box ?slot ?box))
                (free ?slot)
            ))
            (at end (and 
                (contain ?workstation ?box)

            ))
        )
    )
    
    
    (:durative-action deliver-to-location
        :parameters (?agent - agent ?carrier - carrier ?slot - slot ?box - box ?location - location)
        :duration (= ?duration 2)
        :condition (and 
            (at start (and 
                (slot-has-box ?slot ?box)
            ))
            (over all (and 
                (at ?agent ?location)
                (agent-has-carrier ?agent ?carrier)
                (carrier-has-slot ?carrier ?slot)
            ))
        )
        :effect (and 
            (at start (and 
                (not (slot-has-box ?slot ?box))
                (free ?slot)
            ))
            (at end (and 
                (at ?box ?location)
            ))
        )
    )
)