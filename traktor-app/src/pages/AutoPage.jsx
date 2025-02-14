import { useState } from "react";
import TractorForm from "../components/TractorForm";
import LoadingComp from "../components/LoadingComp";
import { useNavigate } from "react-router-dom"; // Untuk navigasi ke halaman lain

function AutoPage() {
  const [isProcessing, setIsProcessing] = useState(false);
  const [isComplete, setIsComplete] = useState(false);
  const navigate = useNavigate();

  const handleFormSubmit = async (tractorPosition, fieldPoints) => {
    setIsProcessing(true); // Menandakan proses dimulai

    try {
      // Mengirim data ke Flask API
      const response = await fetch("http://localhost:5000/process_data", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ tractorPosition, fieldPoints }),
      });

      if (response.ok) {
        const result = await response.json();
        console.log(result);
        setIsProcessing(false);
        setIsComplete(true);
        
        // Beralih ke beranda setelah proses selesai
        setTimeout(() => {
          navigate("/map"); // Mengarahkan ke halaman beranda
        }, 2000);
      } else {
        throw new Error("Gagal mengirim data ke server");
      }
    } catch (error) {
      console.error(error);
      setIsProcessing(false);
    }
  };

  return (
    <div className="mt-6">
      {!isComplete ? (
        <>
          <TractorForm onSubmit={handleFormSubmit} />
          {isProcessing && (
            <div className="flex justify-center items-center mt-4">
              <LoadingComp/>
              <span className="ml-4 text-lg font-semibold">Mohon Tunggu...</span>
            </div>
          )}
        </>
      ) : (
        <div className="flex justify-center items-center mt-4">
          <h2 className="text-xl font-semibold text-green-500">Proses Selesai!</h2>
        </div>
      )}
    </div>
  );
}

export default AutoPage;
