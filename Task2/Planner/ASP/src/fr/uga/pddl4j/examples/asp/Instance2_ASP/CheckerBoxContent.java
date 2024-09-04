package fr.uga.pddl4j.examples.asp;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.lang.StringBuilder;
import java.util.Objects;
import java.util.HashSet;

public class CheckerBoxContent {
    public HashMap<String, List<String>> boxLocation = new HashMap<>();   // Mappa delle locazioni alle liste di scatole
    public HashMap<String, List<String>> contentLocation = new HashMap<>(); // Mappa delle locazioni alle liste di contenuti
    public HashMap<String,String> boxContent = new HashMap<>(); //Mappa delle posizioni dei contenuti nei box
    public Set<String> processedContent = new HashSet<>(); //Set dei contenuti processati
    public Set<String> processedBox = new HashSet<>(); //Set delle box processate

    public HashMap<String, String> contentType;  // Mappa che associa ogni contenuto al suo tipo
    public Set<String> locations = new HashSet<>();  // Lista di tutte le locazioni conosciuteì

    public CheckerBoxContent() {
    }

    public void setContentType(HashMap<String, String> contentType){
        this.contentType = contentType;
    }

     public CheckerBoxContent(CheckerBoxContent other) {
        this.contentType = other.contentType;

        // Deep copy delle mappe e delle liste
        for (String loc : other.boxLocation.keySet()) {
            this.boxLocation.put(loc, new LinkedList<>(other.boxLocation.get(loc)));
        }

        for (String loc : other.contentLocation.keySet()) {
            this.contentLocation.put(loc, new LinkedList<>(other.contentLocation.get(loc)));
        }
        for (String box : other.boxContent.keySet()) {
            this.boxContent.put(box, other.boxContent.get(box));
        }
        for (String processedContentElement : other.processedContent){
            this.processedContent.add(processedContentElement);
        }
        for (String processedBoxElement : other.processedBox){
            this.processedBox.add(processedBoxElement);
        }

        this.locations = new HashSet<>(other.locations);
    }

    // Funzione per aggiungere un oggetto a una locazione specifica
    public void insert(String object, String location, String type) {
        switch (type) {
            case "BOX":
                addObjectToLocation(boxLocation, object, location);
                break;
            case "CONTENT":
                addObjectToLocation(contentLocation, object, location);
                break;
            default:
                System.err.println("Tipo non riconosciuto: " + type);
                break;
        }
        addLocationIfAbsent(location);
    }

    public void updateBoxLocation(String box, String location, boolean deploy){
        List<String> boxes = boxLocation.get(location);

        if(deploy){ //Caso in cui l'agente lascia la scatola
            if(boxes==null){
                List<String> boxesList = new LinkedList<>();
                boxLocation.put(location,boxesList);
            }
            boxLocation.get(location).add(box);
        }
        else{ //Caso in cui l'agente fa la pickup della scatola
            boxLocation.get(location).remove(box);
            processedBox.add(box);
        }

    }

    public void updateContentLocation(String box, String location, String content, boolean empty, boolean isWorkstation){
        List<String> contents = contentLocation.get(location);
        //System.out.println(box+" . "+location+" . "+content);

        if(empty){ //Caso della empty del contenuto nella workstation
            processedBox.remove(box);
            if (boxContent.containsKey(box)) {
                boxContent.remove(box);
            }
            //Nel caso della delivery nella workstation il contenuto, dato il pruning, 
            //soddisferà sicuramente il goal quindi non lo aggiungiamo alla location
            //In caso di empty in una location allora deve essere inserito
            if(!isWorkstation){
                if(contentLocation.get(location)==null){
                    List<String> contentsList = new LinkedList<>();
                    contentLocation.put(location,contentsList);
                }
                contentLocation.get(location).add(content);
            }
        }   
        else{ //Caso di fill
            if(contentLocation.containsKey(location)){
                contentLocation.get(location).remove(content);
                processedContent.add(content);
                //System.out.println("processedContent: "+processedContent);
                boxContent.put(box,content);
            }
        }

    }

    public int getProcessedContentHash() {
        return processedContent.hashCode();
    }

    public int getProcessedBoxHash(){
        return processedBox.hashCode();
    }

    // Aggiunge un oggetto a una lista di locazioni specifiche
    private void addObjectToLocation(HashMap<String, List<String>> locationMap, String object, String location) {
        List<String> objects = locationMap.get(location);
        if (objects == null) {
            objects = new LinkedList<>();
            locationMap.put(location, objects);
        }

        locations.add(location);
        objects.add(object);
    }

    private void updateObjectLocationBox(HashMap<String, List<String>> locationMap, String object, String location, boolean deploy) {
        List<String> objects = locationMap.get(location);
        if (objects == null) {
            objects = new LinkedList<>();
            locationMap.put(location, objects);
        }
        if (deploy) { //Caso di Deploy della scatola
            objects.add(object);
            locations.add(location);
        } 
        else {
            objects.remove(object);
        }
    }

    // Aggiunge una locazione alla lista delle locazioni conosciute se non è già presente
    private void addLocationIfAbsent(String location) {
        if (!locations.contains(location)) {
            locations.add(location);
        }
    }

    public boolean checkPresence(Set<String> type) {
        for (String loc : locations) {
            if (boxLocation.containsKey(loc) && contentLocation.containsKey(loc)) {
                if (!boxLocation.get(loc).isEmpty() && !contentLocation.get(loc).isEmpty()) { 
                    List<String> contents = contentLocation.get(loc);
                    for (String content : contents) {
                        if (type.contains(contentType.get(content))) {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    public String foundContent(String content) {
        for (String location : contentLocation.keySet()) {
            if (contentLocation.get(location).contains(content)) {
                return location;
            }
        }
        return null;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("BOX \n");
        sb.append(boxLocation.toString()).append("\n");
        sb.append("CONTENT \n");
        sb.append(contentLocation.toString()).append("\n");
        sb.append("LOCATION \n");
        sb.append(locations.toString()).append("\n");
        sb.append("BOXCONTENT \n");
        sb.append(boxContent.toString()).append("\n");
        sb.append("PROCESSED CONTENT \n");
        sb.append(processedContent.toString()).append("\n");
        sb.append("PROCESSED BOX \n");
        sb.append(processedBox.toString()).append("\n");
        return sb.toString();
    }
}
