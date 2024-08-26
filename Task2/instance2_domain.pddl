(define (domain task1domain)

  (:requirements :strips :typing :universal-preconditions :disjunctive-preconditions)

  (:types
    workstation location box content robot content_type 
    
    carrier number
    

    
    workstation box content agent - locable
    box workstation carrier - container
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

        ;modifiche per istanza no. 2
        (capacity ?carrier - carrier ? ?value - number)                   ;definizione capacità del carrier
        (curr-carrier-load ?carrier - carrier ?value - number)            ;tiene traccia del numero CORRENTE di scatole sul carrier (richiesto dalla traccia)
        (full-carrier ?carrier)                                           ;true se carrier pieno
        (not-full-carrier ?carrier)                                       ;true se carrier ha almeno un posto vuoto
        (carrier-has-box ?carrier - carrier ?box - box)                   ;true se carrier ha box
        (agent-has-carrier ?agent - agent ?carrier - carrier)             ;true se agent ha il carrier

    )



    ; 1: ogni agente ha un "carrier" con una capacità massima (può essere diversa per ogni agente)       ;DONE
    ; 2: l'agente può caricare le scatole nel "carrier" fino alla capacità massima
    ; 3: l'agente e le scatole da caricare devono essere nella stessa location
    ; 4: l'agente può muovere il carrello ad una locazione dove le ws hanno bisogno dei materiali
    ; 5: l'agente può scaricare una o più scatole dal "carrier" alla locazione dove si trova
    ; 6: l'agente può continuare a muoversi verso un'altra locazione, scaricare altre scatole senza tornare al warehouse ogni volta
    ; 7: diversi tipi di "carrier" 
    ; 8: per ogni agente bisogna contare e tenere traccia di: i) quali scatole sono sul suo "carrier" ii) quante scatole ci sono sul carrier (per non superare la capacità massima) ;DONE
    ; 9: la capacità del "carrier" deve essere "problem specific", quindi deve essere definita nel file del problema.                                                               ;DONE
    ;CHECK IF NUMBER IS USEFUL

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
        :parameters (?agent - agent ?carrier - carrier ?maxCap - number ?currLoad - number ?box - box ?workstation - workstation ?loc - location)
        :precondition (and 
                        (at ?agent ?loc)
                        (at ?workstation ?loc)
                        (contain ?workstation ?box)
                        (free ?agent)
                        ;modifiche istanza no.2
                        (agent-has-carrier ?agent ?carrier)
                        (not-full-carrier ?carrier)
                        (curr-carrier-load ?carrier currLoad)
                        (capacity ?carrier ?maxCap )
        )
        :effect (and 
                (not (free ?agent))
                (not (contain ?workstation ?box))
                ;(loaded ?agent ?box)
                (carrier-has-box ?carrier ?box)
                (not (curr-carrier-load ?carrier currLoad)) ;non è vero perché devo aumentarlo di 1
                (curr-carrier-load ?carrier currLoad+1)            
        )
    )


)

