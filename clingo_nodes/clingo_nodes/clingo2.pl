% Ricetta Dishes (same as in your original code)
ricetta(crostini_vegetali, antipasto, 250, 5, 40, 8).
ricetta(zucchine_grigliate, contorno, 120, 2, 10, 5).
ricetta(frutta_fresca, dessert, 120, 1, 30, 0).
ricetta(sorbetto_al_limone, dolce, 150, 1, 35, 0).
ricetta(torta_alle_mele, dolce, 300, 4, 60, 10).
ricetta(quinoa_con_verdure, piatto_unico, 400, 15, 70, 10).
ricetta(pasta_al_pesto, primo, 450, 12, 85, 5).
ricetta(risotto_ai_funghi, primo, 380, 8, 70, 4).
ricetta(minestrone_di_verdure, primo, 300, 10, 60, 5).
ricetta(vellutata_di_zucca, primo, 200, 4, 40, 6).
ricetta(crema_di_zuppa_di_pomodoro, primo, 200, 3, 25, 5).
ricetta(insalata_di_riso, primo, 320, 10, 60, 5).
ricetta(zuppa_di_fagioli, primo, 350, 15, 50, 10).
ricetta(branzino_al_forno, secondo, 250, 30, 0, 8).
ricetta(frittata_di_zucchine, secondo, 280, 15, 5, 12).
ricetta(pollo_alla_griglia, secondo, 320, 35, 0, 8).
ricetta(bistecca_alla_fiorentina, secondo, 600, 50, 0, 30).
ricetta(filetto_di_manzo, secondo, 400, 45, 0, 15).
ricetta(pollo_al_curry, secondo, 450, 40, 5, 15).
ricetta(spezzatino_con_patate, secondo, 500, 35, 30, 20).

% Current dishes (this would be the user's selected dishes)
% This would be a hard-coded or dynamically input set of current dishes
current_dishes(pasta_al_pesto).
current_dishes(risotto_ai_funghi).
current_dishes(frittata_di_zucchine).

% Total nutrients for current dishes
totale(Nutriente, Somma) :-
    current_dishes(Dish),
    nutrienti(Dish, Nutriente, Valore),
    Somma = #sum { Valore : current_dishes(Dish) }.

% Nutrient association for dishes
nutrienti(Ricetta, calorie, Calorie) :- ricetta(Ricetta, _, Calorie, _, _, _).
nutrienti(Ricetta, proteine, Proteine) :- ricetta(Ricetta, _, _, Proteine, _, _).
nutrienti(Ricetta, carboidrati, Carboidrati) :- ricetta(Ricetta, _, _, _, Carboidrati, _).
nutrienti(Ricetta, grassi, Grassi) :- ricetta(Ricetta, _, _, _, _, Grassi).

% Available dishes (these are the dishes the user can choose from)
% These dishes can be any of the available ones
available_dishes(Dish) :- ricetta(Dish, _, _, _, _, _).

% Substitution rule: find a combination of available dishes that meets the nutritional needs of the current dishes
0 { selezionata(Dish) : available_dishes(Dish) } N :-
    N = #count { Dish : current_dishes(Dish) }.

% Calculate total nutrients from selected substitute dishes
totale_substitute(Nutriente, Somma) :-
    selezionata(Dish),
    nutrienti(Dish, Nutriente, Valore),
    Somma = #sum { Valore : selezionata(Dish) }.

% Remaining nutrients after substitution
rimanenti(Nutriente, Resto) :-
    totale(Nutriente, Totale), % from current dishes
    totale_substitute(Nutriente, SubstituteTotale),
    Resto = Totale - SubstituteTotale.

% Check if the remaining nutrient difference exceeds 10% (threshold)
:- rimanenti(Nutriente, Resto), Resto > (Totale * 10 / 100).
:- rimanenti(Nutriente, Resto), Resto < (-Totale * 10 / 100).

% Minimization of the absolute value of remaining nutrients
#minimize { |Resto| : rimanenti(Nutriente, Resto) }.
