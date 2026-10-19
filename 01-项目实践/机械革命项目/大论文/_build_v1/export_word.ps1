param([string]$InputDoc,[string]$OutputPdf,[switch]$Update)
$ErrorActionPreference='Stop'
$word=New-Object -ComObject Word.Application
$word.Visible=$false
$word.DisplayAlerts=0
$word.Options.SaveNormalPrompt=$false
$doc=$null
try {
 $doc=$word.Documents.Open($InputDoc,$false,(!$Update),$false)
 if($Update){
  [void]$doc.Fields.Update()
  foreach($toc in $doc.TablesOfContents){$toc.Update()}
  foreach($styleId in @(-20,-21,-22)){
   $st=$doc.Styles.Item($styleId)
   $st.Font.Name='Times New Roman'
   $st.Font.NameFarEast='宋体'
   $st.Font.Size=10.5
   $st.Font.Color=0
   $st.Font.Bold=0
   $st.ParagraphFormat.SpaceBefore=0
   $st.ParagraphFormat.SpaceAfter=3
   $st.ParagraphFormat.LineSpacingRule=0
   $st.ParagraphFormat.LeftIndent=12*(-20-$styleId)
   $st.ParagraphFormat.FirstLineIndent=0
   $st.ParagraphFormat.RightIndent=0
   $st.ParagraphFormat.TabStops.ClearAll()
   [void]$st.ParagraphFormat.TabStops.Add(453.54,2,1)
  }
  foreach($toc in $doc.TablesOfContents){
   foreach($para in $toc.Range.Paragraphs){
    $para.Format.TabStops.ClearAll()
    [void]$para.Format.TabStops.Add(453.54,2,1)
    $para.Format.RightIndent=0
   }
  }
  $doc.Repaginate()
  foreach($toc in $doc.TablesOfContents){$toc.UpdatePageNumbers()}
  $doc.Save()
 }
 $doc.ExportAsFixedFormat($OutputPdf,17)
 [PSCustomObject]@{Pages=$doc.ComputeStatistics(2);Words=$doc.ComputeStatistics(0);Tables=$doc.Tables.Count;Pictures=$doc.InlineShapes.Count;Equations=$doc.OMaths.Count;Pdf=$OutputPdf}|ConvertTo-Json
} finally {
 if($null -ne $doc){$doc.Close(0);[void][Runtime.InteropServices.Marshal]::ReleaseComObject($doc)}
 $word.Quit(0)
 [void][Runtime.InteropServices.Marshal]::ReleaseComObject($word)
}
