%%
%% Copyright (C) 2010 by Karinne Ramirez-Amaro
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%
/* ***************************************
	Author:	Karinne Ramirez-Amaro
	E-mail:	karinne@chalmers.se
	
 This library contains predicates used for the
 inference method using prolog queries.


 NOTE: The following symbols are used to describe the parameter
of the predicates
 + The argument is instantiated at entry.
 - The argument is not instantiated at entry.
 ? The argument is unbound or instantiated at entry.

*/

:- module(instance_utils,
    [
	create_instance_from_class/3,
	getClassPath/2,
	get_class/1
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_db:rdf_register_ns(ssy236Ontology, 'http://www.chalmers.se/ontologies/ssy236Ontology.owl#', [keep(true)]).

%%%%%%%%%%%%%% Custom computables %%%%%%%%%%%%%%%%%%%%%%

% This function will create an instance of a desired class
% create_instance_from_class(+Class, +Instance_ID, ?Instance)
% The created instance will have the predicate/property rdf:type
% to correctly inheritate the properties of its super-classes
%
% @param Class		represents the name of the class where the instance will be created.
%					Class could be of two forms:
%					Class='Orange'  <- make sure this class exist in the ontology "ssy236Ontology"
% @param Instance_ID	is the new ID that the instance will have
% @param Instance	asserted new instance

% TODO: This prolog query should check if the class we want to insert exist in the ontology. IF the class does not exits it should create a parent class, then the new instance.

create_instance_from_class(Class, Instance_ID, Instance) :-
	% Check ID and class path
	getClassPath(Class,Class_path),
	
	% Create the path of the new instance
	atom_concat(Class_path,  '_', Class2),
	atomic_concat(Class2, Instance_ID, Instance),
	% write(Instance),nl,
	% assert/create the new instance
	rdf_assert(Instance, rdf:type, Class_path).

% This function will return the path of the class/instance given
% getClassPath(+Class, ?Class_path)
%
% @param Class		represents the name of the class where the instance// Get object location will be created.
%					Class could be of two forms:
%					Class='Orange'  <- make sure this class exist in the ontology "ssy236Ontology"
% @param Class_path	correct class full path

 getClassPath(Class, Class_path):-
 	((concat_atom(List, '#', Class),length(List,Length),Length>1) ->
 	( % Class has already a URI
 	   Class_path=Class );
 	  % When the class does not have URL, will get the knowrob path
         ( TempClass='http://www.chalmers.se/ontologies/ssy236Ontology.owl#',
 	atom_concat(TempClass, Class, Class_path)
 	% write(Class_path), nl
  	)).


% This function will return the path of the obtained class
% get_class(+Class)
%
% @param Class		represents the name of the class that we ask to create (if it does not exsist yet).
%					Class could be of two forms:
%					Class='Orange'  <- make sure this class exist in the ontology "ssy236Ontology"

% If Class already exists, do nothing
% If Class do not exist, add the class

get_class(Class):-
	getClassPath(Class, Class_path),
	(
		rdf_has(Class_path, rdf:type, owl:'Class') ->
		false;  
		rdf_assert(Class_path, rdf:type, owl:'Class'),
		write('New class created: '), write(Class_path), nl
	).


%  four new Prolog predicates to infer more knowledge about the environment or the task.

% Find what surface an object is on
surface_of_object(Object, Surface) :-
    rdf_has(Surface, ssy236Ontology:'objectActedOn', Object),
    rdf_has(Surface, rdf:type, SurfaceType),
% The immediate super class of surface should be of type furniture or container
    (rdf_has(SurfaceType, rdfs:subClassOf, ssy236Ontology:'Furniture');
     rdf_has(SurfaceType, rdfs:subClassOf, ssy236Ontology:'Container')).

% find the shape of the object
shape_of_object(Object, Shape) :-
    rdf_has(Object, rdf:type, Shape),
    rdf_has(Shape, rdfs:subClassOf, ssy236Ontology:'ThingTypeShape').

% find all the edible objects
find_edible_objects(EdibleObjects) :-
% finds all instances of subclasses of EdibleStuffs
    findall(Object, 
        (rdf_has(Object, rdf:type, Class),
         rdf_has(Class, rdfs:subClassOf, EdibleClass),
         rdf_has(EdibleClass, rdfs:subClassOf, ssy236Ontology:'EdibleStuff')),
    EdibleObjects).

% find all objects sharing same shape
objects_same_shape(Object, SimilarObjects) :-
    % find the class of the object
    rdf_has(Object, rdf:type, ShapeClass),
    % verify whether a shape class
    rdf_has(ShapeClass, rdfs:subClassOf, ssy236Ontology:'ThingTypeShape'),
    % Find other objects of same shape
    findall(Similar, (
        rdf_has(Similar, rdf:type, ShapeClass),
        Similar \= Object
    ), SimilarObjects).

% three new predicates for decision making

% decides if object is edible
is_edible(Object) :-
    rdf_has(Object, rdf:type, Class),
    rdf_has(Class, rdfs:subClassOf, EdibleClass),
    rdf_has(EdibleClass, rdfs:subClassOf, ssy236Ontology:'EdibleStuff').

% returns true if obj2 has similar shape and on same surface as obj1
is_similar_shape_nearby(Object1, Object2) :-
    objects_same_shape(Object1, SimilarObjects),
    member(Object2, SimilarObjects),
    surface_of_object(Object1, Surface),
    surface_of_object(Object2, Surface).

% returns true if surface has any edible objects
is_edible_nearby(Surface) :-
    find_edible_objects(EdibleObjects),
    member(EdibleObj, EdibleObjects),
    surface_of_object(EdibleObj, Surface).