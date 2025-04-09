% --- Define nutrients
nutriente(calorie).
nutriente(proteine).
nutriente(carboidrati).
nutriente(grassi).

% --- Recipe database
ricetta(crostini_vegetali, antipasto, 250, 5, 40, 8).
ricetta(zucchine_grigliate, contorno, 120, 2, 10, 5).
ricetta(frutta_fresca, dessert, 120, 1, 30, 0).
ricetta(sorbetto_al_limone, dolce, 150, 1, 35, 0).
ricetta(torta_alle_mele, dolce, 300, 4, 60, 10).
ricetta(quinoa_con_verdure, piatto_unico, 400, 15, 70, 10).
ricetta(minestrone_di_verdure, primo, 300, 10, 60, 5).
ricetta(vellutata_di_zucca, primo, 200, 4, 40, 6).
ricetta(crema_di_zuppa_di_pomodoro, primo, 200, 3, 25, 5).
ricetta(insalata_di_riso, primo, 320, 10, 60, 5).
ricetta(zuppa_di_fagioli, primo, 350, 15, 50, 10).
ricetta(branzino_al_forno, secondo, 250, 30, 0, 8).
ricetta(pollo_alla_griglia, secondo, 320, 35, 0, 8).
ricetta(bistecca_alla_fiorentina, secondo, 600, 50, 0, 30).
ricetta(filetto_di_manzo, secondo, 400, 45, 0, 15).
ricetta(pollo_al_curry, secondo, 450, 40, 5, 15).
ricetta(spezzatino_con_patate, secondo, 500, 35, 30, 20).

% --- Current dishes chosen by user
current_ricetta(pasta_al_pesto, primo, 450, 12, 85, 5).
current_ricetta(risotto_ai_funghi, primo, 380, 8, 70, 4).
current_ricetta(frittata_di_zucchine, secondo, 280, 15, 5, 12).


% --- Nutrient extractor
nutrienti_desiderati(R, calorie, C)     :- current_ricetta(R, _, C, _, _, _).
nutrienti_desiderati(R, proteine, P)    :- current_ricetta(R, _, _, P, _, _).
nutrienti_desiderati(R, carboidrati, Ca):- current_ricetta(R, _, _, _, Ca, _).
nutrienti_desiderati(R, grassi, G)      :- current_ricetta(R, _, _, _, _, G).

% --- Total desired nutrients (safe!)
totale_nutrienti_desiderati(Nutriente, Somma) :-
    nutriente(Nutriente),
    Somma = #sum { Valore,R : nutrienti_desiderati(R, Nutriente, Valore) }.


% Scelta delle ricette: seleziona fino a 4 piatti
0 { selezionata(Ricetta, Tipo) : ricetta(Ricetta, Tipo, _, _, _, _) } N :-
    N = #count { Tipo : ricetta(_, Tipo, _, _, _, _) }.

% Associazione nutrienti ai piatti
nutrienti_per_piatto(Ricetta, calorie, Calorie) :- ricetta(Ricetta, _, Calorie, _, _, _).
nutrienti_per_piatto(Ricetta, proteine, Proteine) :- ricetta(Ricetta, _, _, Proteine, _, _).
nutrienti_per_piatto(Ricetta, carboidrati, Carboidrati) :- ricetta(Ricetta, _, _, _, Carboidrati, _).
nutrienti_per_piatto(Ricetta, grassi, Grassi) :- ricetta(Ricetta, _, _, _, _, Grassi).

% Calcolo totale dei nutrienti
totale(Nutriente, Somma) :-
    nutriente(Nutriente),
    Somma = #sum { Valore : selezionata(Ricetta, _), nutrienti_per_piatto(Ricetta, Nutriente, Valore) }.


% Calcolo dei nutrienti rimanenti
rimanenti(Nutriente, Resto) :-
    totale_nutrienti_desiderati(Nutriente, Fabbisogno),
    totale(Nutriente, Totale),
    Resto = Fabbisogno - Totale.


:- rimanenti(Nutriente, Resto), totale_nutrienti_desiderati(Nutriente, Fabbisogno), Resto > (Fabbisogno * 10 / 100).
:- rimanenti(Nutriente, Resto), totale_nutrienti_desiderati(Nutriente, Fabbisogno), Resto < (-Fabbisogno * 10 / 100).

% Minimizzazione del valore assoluto dei nutrienti rimanenti
#minimize { |Resto| : rimanenti(Nutriente, Resto) }.

% --- Output
traccia(selezionata, Ricetta) :- selezionata(Ricetta, Tipo).
#show totale_nutrienti_desiderati/2.
#show traccia/2.
#show totale/2.
#show rimanenti/2.
