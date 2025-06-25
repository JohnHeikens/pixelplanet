#pragma once
enum class PlacementMode {
	Free,//place molecules in any direction
	Orthogonal,//place molecules in right angles from the attachments of the 'parent' molecule
	count
};

//attach new molecules to...
enum class AttachmentMode {
	None,//nothing
	ClickedOn,//the molecule which was clicked on
	Near,//all molecules which are in range
	count
};