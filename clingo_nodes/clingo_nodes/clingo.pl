


% Scelta delle ricette: seleziona fino a 4 piatti
0 { selezionata(Ricetta, Tipo) : ricetta(Ricetta, Tipo, _, _, _, _) } N :-
    N = #count { Tipo : ricetta(_, Tipo, _, _, _, _) }.

% Calcolo totale dei nutrienti
totale(Nutriente, Somma) :-
    fabbisogno(Nutriente, _),
    Somma = #sum { Valore : selezionata(Ricetta, _), nutrienti(Ricetta, Nutriente, Valore) }.


% Associazione nutrienti ai piatti
nutrienti(Ricetta, calorie, Calorie) :- ricetta(Ricetta, _, Calorie, _, _, _).
nutrienti(Ricetta, proteine, Proteine) :- ricetta(Ricetta, _, _, Proteine, _, _).
nutrienti(Ricetta, carboidrati, Carboidrati) :- ricetta(Ricetta, _, _, _, Carboidrati, _).
nutrienti(Ricetta, grassi, Grassi) :- ricetta(Ricetta, _, _, _, _, Grassi).

% Calcolo dei nutrienti rimanenti
rimanenti(Nutriente, Resto) :-
    fabbisogno(Nutriente, Fabbisogno),
    totale(Nutriente, Totale),
    Resto = Fabbisogno - Totale.


:- rimanenti(Nutriente, Resto), fabbisogno(Nutriente, Fabbisogno), Resto > (Fabbisogno * 20 / 100).
:- rimanenti(Nutriente, Resto), fabbisogno(Nutriente, Fabbisogno), Resto < (-Fabbisogno * 20 / 100).

% Minimizzazione del valore assoluto dei nutrienti rimanenti
#minimize { |Resto| : rimanenti(Nutriente, Resto) }.




traccia(selezionata, Ricetta) :- selezionata(Ricetta, Tipo).

#show traccia/2.
#show totale/2.
#show rimanenti/2.