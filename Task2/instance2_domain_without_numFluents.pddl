(define (domain task22domain)

  (:requirements :strips :typing :disjunctive-preconditions)

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
        (connected ?location1 - location ?location2 - location)
        (workstation-has-type ?workstation - workstation ?type - content_type) ; Indica di quale tipo di contenuto ha bisogno una workstation
        (is-type ?content - content ?content_type - content_type); Associa contenuti specifici ai loro tipi

        ;modifiche per istanza no. 2
        (agent-has-carrier ?agent - agent ?carrier - carrier)             ;true se agent ha il carrier
        (carrier-has-slot ?carrier -carrier ?slot - slot)
        (slot-has-box ?slot - slot ?box - box)
        (free ?slot - slot)
    )



    ; 1: ogni agente ha un "carrier" con una capacità massima (può essere diversa per ogni agente)       ;DONE
    ; 2: l'agente può caricare le scatole nel "carrier" fino alla capacità massima
    ; 3: l'agente e le scatole da caricare devono essere nella stessa location
    ; 4: l'agente può muovere il carrello ad una locazione dove le workstation hanno bisogno dei materiali
    ; 5: l'agente può scaricare una o più scatole dal "carrier" alla locazione dove si trova
    ; 6: l'agente può continuare a muoversi verso un'altra locazione, scaricare altre scatole senza tornare al warehouse ogni volta
    ; 7: diversi tipi di "carrier" 
    ; 8: per ogni agente bisogna contare e tenere traccia di: i) quali scatole sono sul suo "carrier" ii) quante scatole ci sono sul carrier (per non superare la capacità massima) ;DONE
    ; 9: la capacità del "carrier" deve essere "problem specific", quindi deve essere definita nel file del problema.                                                               ;DONE


    ;l'assunzione fatta per l'istanza 1 : robot caricato con una scatola ha le "mani" occupate quindi non può eseguire empty e fill, cade in questo caso.
    ;si suppone, visto l'utilizzo di un "carrier", che il robot può eseguire le operazioni di empty e fill
    ;first try: eliminato il predicato free

    (:action fill_box_from_location
        :parameters (?agent - agent ?box - box ?content - content ?location - location )
        :precondition (and 
                        (at ?agent ?location )
                        (at ?box ?location )
                        (at ?content ?location )
                        ;(free ?agent)
                        (box-is-empty ?box)
        )      
        :effect (and 
                    (not (box-is-empty ?box))
                    (not (at ?content ?location ))
                    (contain ?box ?content)
        )
    )

    (:action fill_box_from_workstation
        :parameters (?agent - agent ?box - box ?content - content ?workstation - workstation ?type - content_type ?location - location)
        :precondition (and 
                        (at ?agent ?location)
                        (at ?workstation ?location)
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
    (:action empty-box-workstation
        :parameters (?agent - agent ?box - box ?content - content ?content_type - content_type ?workstation - workstation ?location - location)
        :precondition (and 
                            (at ?agent ?location)
                            (or (at ?box ?location) (contain ?workstation ?box))
                            (at ?workstation ?location)

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

    (:action empty-box-location
        :parameters (?agent - agent ?box - box ?content - content ?location - location)
        :precondition (and 
                            (at ?agent ?location)
                            (at ?box ?location) 
        
                            ;(free ?agent)
                            (contain ?box ?content)
        )
        :effect (and 
                    (not (contain ?box ?content))
                    (box-is-empty ?box)

                    (at ?content ?location)
        )
    )

    (:action pick-up-from-workstation
        :parameters (?agent - agent ?carrier - carrier ?slot - slot ?box - box ?workstation - workstation ?location - location)
        :precondition (and 
                    (at ?agent ?location)
                    (at ?workstation ?location)
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
        :parameters (?agent - agent ?carrier - carrier ?slot - slot ?box - box ?location - location)
        :precondition (and 
                            (at ?agent ?location)
                            (at ?box ?location)
                            (agent-has-carrier ?agent ?carrier)
                            (carrier-has-slot ?carrier ?slot)
                            (free ?slot)

        )
        :effect (and 
                    (not (at ?box ?location))
                    (not (free ?slot))
                    (slot-has-box ?slot ?box)
        )
    )

    (:action move
        :parameters (?agent - agent ?location1 - location ?location2 - location)
        :precondition (and 
                        (or (connected ?location1 ?location1) (connected ?location1 ?location1))
                        (at ?agent ?location1)
        )
        :effect (and 
                    (not (at ?agent ?location1))
                    (at ?agent ?location1)
        )
    )

    (:action deliver-to-workstation
        :parameters (?agent - agent ?carrier - carrier ?slot - slot ?box - box ?workstation - workstation ?location - location)
        :precondition (and 
                            (at ?agent ?location)
                            (at ?workstation ?location)
                            
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

    (:action deliver-to-location
        :parameters (?agent - agent ?carrier - carrier ?slot - slot ?box - box ?location - location)
        :precondition (and 
                            (at ?agent ?location)
                            (agent-has-carrier ?agent ?carrier)
                            (carrier-has-slot ?carrier ?slot)
                            (slot-has-box ?slot ?box)
        )
        :effect (and 
                    (not (slot-has-box ?slot ?box))
                    (free ?slot)
                    (at ?box ?location)
        )
    )

)

