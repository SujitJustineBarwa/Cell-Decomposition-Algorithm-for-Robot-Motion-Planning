# Cell Decomposition Algorithm for Robot Motion Planning
 Cell Decomposition algorithm is a technique in which we decompose the workspace (2D Shape) into quadilateral Cells which are then treated as nodes and using BFS or DFS algorithm we can find the path from Start to End.
 In the provided Code,two methods of doing this is shown Naive and Sweep strategy.The sweep is faster than the naive approach.
 Both the Strategies are implemented and are commented out well.
 Special Thanks to Atul Thakur Sir(from IIT Patna).
 
 # Problem Statement :
\
A.Take input from user using ginput command in matlab(take input(as point)from matlab plot when you click in it.Use these input these plots to obstacles and workspace.\
B.Write a condition to ensure that the X coordinates of the vertices are unique.\
C.Perform cell decomposition using Naïve approach. Store the cells in a suitable datastructure.\
D.Improve the cell decomposition code with sweeping algorithm.\
E.Compare the time taken for cell decomposition suing both Naïve and sweeping algorithm approaches using tic and toc in matlab.

# The approach :
\
**Task 1 :**
\
Ensure all x are unique
The array X stores the x-coordinate of all the vertices.
if length of X is not equal to length of unique(X) then stop.
\

**Task 2 :** 
\
Naiive Strategy
Cell Decomposition using Naiive Strategy:-
All the edges are stored in all_edges as structure array,each  element containing the initial and final coordinate points.
A ployshape(pygon) is made as a configuration space minus obstacle.
A loop is started moving through the X array (loop1).

**a) Collecting the intersection points :**
A  2nd loop starts moving through all_edges to check the edges intersecting vertical extension 
A Table T stores all the points and the edges that are connected to the current extension respectively.
The 2nd loop is stopped
Note : these two loops are what make this whole program an O(n^2) time complex.

**b) Processing on T :**
The point immediate upper to current vertices and immediate below to current vertices are kept and everything else is deleted.
The upper mid point and lower mid point are calculated by taking the average of upper intersection and lower intersection.
Isinterior() function is used to determine if the mid points are inside pygon i.e. the Config. Space.



**c) Cell making Process :**
A Table T_Manager is employed as data structure  to manage vertices and edges.It is updated as :
 		T_Manager = [T_Manager;T]

This table has some internal operations to do in the following sequence :
	Transfer all the rows having equal edges to an Temp_T.
	Eliminate all the points having a equal edges.
	Re-adding T.
	Eliminating all the edges having initial and final points on left of x.

**d) Converting the Temp_T to cell :**
It should be kept in mind that the from Temp_two cells can form simlatenously.
The picture below depicts the this case.
<p align="center">
  <img src="https://raw.githubusercontent.com/SujitJustineBarwa/Robot-Motion-Planning-with-Potential-Field-Approach/main/Images/img2.PNG" />
</p>
     	 
**Sample Temp iteration 1**
First the table is sorted on the basis of x column.
Edges of first two rows are searched on the table and respective X and Y Co-ordinates are transferred to cell_vertices array .

<table style="width: 796px; height: 230px;">
<tbody>
<tr style="height: 25px;">
<td style="width: 167.683px; height: 25px;">&nbsp;X Co-ordinate of the Cell Vertex</td>
<td style="width: 403.417px; height: 25px;">&nbsp;Y Co-ordinate of the cell Vertex</td>
<td style="width: 224.1px; height: 25px;">&nbsp;The Edge on which the current Point Lies in</td>
</tr>
<tr style="height: 25px;">
<td style="width: 167.683px; height: 25px;">&nbsp;x4</td>
<td style="width: 403.417px; height: 25px;">&nbsp;y_upper</td>
<td style="width: 224.1px; height: 25px;">&nbsp;e2</td>
</tr>
<tr style="height: 25px;">
<td style="width: 167.683px; height: 25px;">&nbsp;x4</td>
<td style="width: 403.417px; height: 25px;">&nbsp;y4</td>
<td style="width: 224.1px; height: 25px;">&nbsp;e6</td>
</tr>
<tr style="height: 25px;">
<td style="width: 167.683px; height: 25px;">&nbsp;x6</td>
<td style="width: 403.417px; height: 25px;">&nbsp;y6</td>
<td style="width: 224.1px; height: 25px;">&nbsp;e7</td>
</tr>
<tr style="height: 25.9333px;">
<td style="width: 167.683px; height: 25.9333px;">&nbsp;x6</td>
<td style="width: 403.417px; height: 25.9333px;">&nbsp;y_down</td>
<td style="width: 224.1px; height: 25.9333px;">&nbsp;e12</td>
</tr>
<tr style="height: 25px;">
<td style="width: 167.683px; height: 25px;">&nbsp;x7</td>
<td style="width: 403.417px; height: 25px;">&nbsp;y7</td>
<td style="width: 224.1px; height: 25px;">&nbsp;e6</td>
</tr>
<tr style="height: 25px;">
<td style="width: 167.683px; height: 25px;">&nbsp;x7</td>
<td style="width: 403.417px; height: 25px;">&nbsp;y7</td>
<td style="width: 224.1px; height: 25px;">&nbsp;e7</td>
</tr>
<tr style="height: 25px;">
<td style="width: 167.683px; height: 25px;">&nbsp;x7</td>
<td style="width: 403.417px; height: 25px;">&nbsp;y_upper</td>
<td style="width: 224.1px; height: 25px;">&nbsp;e2</td>
</tr>
<tr style="height: 24px;">
<td style="width: 167.683px; height: 24px;">&nbsp;x7</td>
<td style="width: 403.417px; height: 24px;">&nbsp;y_down</td>
<td style="width: 224.1px; height: 24px;">e12</td>
</tr>
</tbody>
</table>
<!-- DivTable.com -->
<p>&nbsp;</p>
 
 
 Cell 1 :
<!-- DivTable.com -->
<p>&nbsp;</p>
<table style="height: 144px;" width="805">
<tbody>
<tr>
<td style="width: 171.333px;">x4</td>
<td style="width: 455.317px;">y_upper</td>
<td style="width: 177.55px;">e2</td>
</tr>
<tr>
<td style="width: 171.333px;">x4</td>
<td style="width: 455.317px;">y4</td>
<td style="width: 177.55px;">e6</td>
</tr>
<tr>
<td style="width: 171.333px;">x7</td>
<td style="width: 455.317px;">y7</td>
<td style="width: 177.55px;">e6</td>
</tr>
<tr>
<td style="width: 171.333px;">x7</td>
<td style="width: 455.317px;">y_upper</td>
<td style="width: 177.55px;">e2</td>
</tr>
</tbody>
</table>
<p>&nbsp;</p>

Cell 2:
<!-- DivTable.com -->
<p>&nbsp;</p>
<table style="height: 144px;" width="805">
<tbody>
<tr>
<td style="width: 171.333px;">x6</td>
<td style="width: 455.317px;">y_upper</td>
<td style="width: 177.55px;">e7</td>
</tr>
<tr>
<td style="width: 171.333px;">x6</td>
<td style="width: 455.317px;">y6</td>
<td style="width: 177.55px;">e12</td>
</tr>
<tr>
<td style="width: 171.333px;">x7</td>
<td style="width: 455.317px;">y7</td>
<td style="width: 177.55px;">e7</td>
</tr>
<tr>
<td style="width: 171.333px;">x7</td>
<td style="width: 455.317px;">y_down</td>
<td style="width: 177.55px;">e12</td>
</tr>
</tbody>
</table>
<p>&nbsp;</p>
**Sample Temp iteration 2**

If cell _vertices array length is 4 they are transferred to a cells array and cell_vertices is reset.
Cells array is datatype cell which stores 4 coordinates at each of its index.

**Task 3 :**  Improvement using Sweep Strategy
All the points are same as above except for step a.

	Step A modified
All the Edges intersecting line x = x1 are recorded and added to the Table L in the tree like fashion. (In the first condition of slide L5 pg.7 of Ref[1]).
A jumper table is kept to record the deleted or changed row in the table L.
(In the last 3 condition of slide L5 pg.7 of Ref[1]).
The jumper and L is added to L_duplicate.
The Intersection point are found out with all the edges in L_duplicate and stored in T as previous format.


Note :  A Problem has been worked out below to have an idea of how T_Manager works after each time we move to new x.The process mentioned above are applied.If the elimination is due to common edges than they are transferred to Temp_T.
